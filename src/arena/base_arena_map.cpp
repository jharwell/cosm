/**
 * \file base_arena_map.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * This file is part of COSM.
 *
 * COSM is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * COSM is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * COSM.  If not, see <http://www.gnu.org/licenses/
 */
/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/arena/base_arena_map.hpp"

#include <algorithm>

#include <argos3/plugins/simulator/media/led_medium.h>

#include "rcppsw/utils/maskable_enum.hpp"

#include "cosm/arena/config/arena_map_config.hpp"
#include "cosm/arena/ds/loctree.hpp"
#include "cosm/arena/free_blocks_calculator.hpp"
#include "cosm/arena/repr/arena_cache.hpp"
#include "cosm/arena/repr/light_type_index.hpp"
#include "cosm/ds/cell2D.hpp"
#include "cosm/ds/operations/cell2D_empty.hpp"
#include "cosm/foraging/block_dist/base_distributor.hpp"
#include "cosm/foraging/block_dist/block3D_manifest_processor.hpp"
#include "cosm/foraging/block_dist/dispatcher.hpp"
#include "cosm/pal/argos/swarm_manager_adaptor.hpp"
#include "cosm/repr/config/nest_config.hpp"
#include "cosm/repr/nest.hpp"
#include "cosm/repr/operations/nest_extent.hpp"
#include "cosm/repr/sim_block3D.hpp"
#include "cosm/spatial/conflict_checker.hpp"
#include "cosm/spatial/dimension_checker.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, arena);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
base_arena_map::base_arena_map(const caconfig::arena_map_config* config,
                               rmath::rng* rng)
    : ER_CLIENT_INIT("cosm.arena.base_arena_map"),
      decorator(config->grid.dims, config->grid.resolution),
      m_rng(rng),
      m_blockso(foraging::block_dist::block3D_manifest_processor(
          &config->blocks.dist.manifest,
          config->grid.resolution)()),
      m_block_dispatcher(
          std::make_unique<cfbd::dispatcher>(&decoratee(),
                                             config->grid.resolution,
                                             &config->blocks.dist)),
      m_redist_governor(&config->blocks.dist.redist_governor),
      m_bm_handler(&config->blocks.motion, m_rng),
      m_nests(std::make_unique<nest_map_type>()),
      m_bloctree(std::make_unique<cads::loctree>()),
      m_nloctree(std::make_unique<cads::loctree>()) {
  ER_INFO("real=(%fx%f), discrete=(%zux%zu), resolution=%f",
          xrsize(),
          yrsize(),
          xdsize(),
          ydsize(),
          grid_resolution().v());

  /*
   * Initialize nests, if configured (nests can also be manually initialized
   * later). The swarm manager is not yet initialized, so we can't do that part
   * of nest initialization later.
   */
  initialize_nests(&config->nests, nullptr, config->grid.resolution);

  /* Initialize non-owning block vector exposed to outside classes */
  for (auto& b : m_blockso) {
    m_blocksno.push_back(b.get());
  } /* for(&b..) */
}

base_arena_map::~base_arena_map(void) = default;

/*******************************************************************************
 * Initialization Functions
 ******************************************************************************/
bool base_arena_map::initialize(cpargos::swarm_manager_adaptor* sm,
                                const crepr::config::nests_config* nests) {
  bool ret = initialize_shared(sm, nests);
  ret |= initialize_private();
  return ret;
} /* initialize */

bool base_arena_map::initialize_shared(cpargos::swarm_manager_adaptor* sm,
                                       const crepr::config::nests_config* nests) {
  /* compute block bounding box */
  auto* block = *std::max_element(
      m_blocksno.begin(), m_blocksno.end(), [&](const auto* b1, const auto* b2) {
        return rmath::vector3d::componentwise_compare()(b1->rdims3D(),
                                                        b2->rdims3D());
      });
  m_block_bb = block->rdims3D();

  /* initialize nests */
  initialize_nests(nests, sm, grid_resolution());

  return true;
} /* initialize_shared() */

bool base_arena_map::initialize_private(void) {
  /* distribute blocks */
  auto avoid_ents = initial_dist_precalc(nullptr);

  auto conflict_check = [&](const crepr::sim_block3D* block,
                            const rmath::vector2d& loc) {
    return cspatial::conflict_checker::placement2D(this, block, loc);
  };
  auto dist_success = [&](const crepr::sim_block3D* distributed) {
    /*
                         * Update block location query tree. This is called from
                         * inside a block distributor, and therefore inside a
                         * context in which all necessary locks have already
                         * been taken.
                         */
    bloctree_update(distributed, locking::ekALL_HELD);
  };

  bool ret = m_block_dispatcher->initialize(
      this, avoid_ents, m_block_bb, conflict_check, dist_success, m_rng);
  ret |= distribute_all_blocks();
  return ret;
} /* initialize_private() */

void base_arena_map::initialize_nests(const crepr::config::nests_config* nests,
                                      cpargos::swarm_manager_adaptor* sm,
                                      const rtypes::discretize_ratio& resolution) {
  if (nullptr != nests) {
    ER_INFO("Initialize %zu nests", nests->nests.size());
    for (crepr::config::nest_config nest : nests->nests) {
      /*
       * Trim nest size if necessary so it is an even multiple of the grid
       * resolution--many parts of the code rely on 2D spatial objects on the
       * arena floor being exact multiples of the grid size.
       *
       * Note that we are NOT updating the dimensions held by the parent arena
       * map configuration.
       */
      nest.dims = cspatial::dimension_checker::even_multiple(grid_resolution(),
                                                             nest.dims);
      crepr::nest inst(&nest, rdims2D(), resolution);
      /* configure nest extent */
      for (size_t i = inst.xdspan().lb(); i <= inst.xdspan().ub(); ++i) {
        for (size_t j = inst.ydspan().lb(); j <= inst.ydspan().ub(); ++j) {
          auto coord = rmath::vector2z(i, j);
          crops::nest_extent_visitor op(coord, &inst);
          op.visit(access<cads::arena_grid::kCell>(coord));
        } /* for(j..) */
      } /* for(i..) */

      /* add to set of arena nests */
      m_nests->emplace(std::make_pair(inst.id(), inst));

      /* add to loctree */
      m_nloctree->update(&inst);
    } /* for(nest...) */
  }

  if (nullptr != sm) {
    /*
     * Initialize nest lights. Might do nothing if the lights have already been
     * initialized.
     */
    for (auto& pair : *m_nests) {
      pair.second.initialize(
          sm, carepr::light_type_index()[carepr::light_type_index::kNest]);
    } /* for(&pair..) */
  }
} /* initialize_nests() */

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
const cfbd::base_distributor* base_arena_map::block_distributor(void) const {
  return m_block_dispatcher->distributor();
}
cfbd::base_distributor* base_arena_map::block_distributor(void) {
  return m_block_dispatcher->distributor();
}
const rmath::rangez& base_arena_map::distributable_cellsx(void) const {
  return m_block_dispatcher->distributable_cellsx();
}
const rmath::rangez& base_arena_map::distributable_cellsy(void) const {
  return m_block_dispatcher->distributable_cellsy();
}
update_status base_arena_map::pre_step_update(const rtypes::timestep&) {
  size_t count = m_bm_handler.move_blocks(this);
  if (count > 0) {
    return update_status::ekBLOCK_MOTION;
  }
  return update_status::ekNONE;
} /* pre_step_update() */

void base_arena_map::post_step_update(const rtypes::timestep& t,
                                      size_t blocks_transported,
                                      bool convergence_status) {
  redist_governor()->update(t, blocks_transported, convergence_status);
} /* post_step_update() */

rtypes::type_uuid
base_arena_map::robot_on_block(const rmath::vector2d& pos,
                               const rtypes::type_uuid& ent_id) const {
  auto ret = rtypes::constants::kNoUUID;
  /*
   * If the robot actually is on the block they think they are, we can short
   * circuit what may be an expensive linear search. ent_id MIGHT be for a
   * non-block that a robot has acuired, which may cause out of bounds indexing
   * into the blocks vector, so we have to check for that.
   */
  if (ent_id != rtypes::constants::kNoUUID &&
      static_cast<size_t>(ent_id.v()) < m_blockso.size() &&
      m_blockso[ent_id.v()]->contains_point(pos)) {
    ret = ent_id;
  } else {
    /* General case: linear scan */
    for (auto& b : m_blockso) {
      if (b->contains_point(pos)) {
        ret = b->id();
      }
    } /* for(&b..) */
  }
  return ret;
} /* robot_on_block() */

rtypes::type_uuid
base_arena_map::robot_in_nest(const rmath::vector2d& pos) const {
  auto it = std::find_if(m_nests->begin(), m_nests->end(), [&](const auto& pair) {
    return pair.second.contains_point(pos);
  });
  if (m_nests->end() == it) {
    return rtypes::constants::kNoUUID;
  }
  return it->second.id();
} /* robot_in_nest() */

void base_arena_map::distribute_single_block(crepr::sim_block3D* block,
                                             const locking& locking) {
  /* The distribution of nothing is ALWAYS successful */
  if (!m_redist_governor.enabled()) {
    return;
  }

  /* lock the arena map */
  pre_block_dist_lock(locking);

  /* block to be distributed is tried before any leftover blocks */
  m_pending_dists.push_front({ block, 0 });

  auto it = m_pending_dists.begin();
  while (m_pending_dists.end() != it) {
    /* do the distribution */
    auto status = m_block_dispatcher->distribute_block(it->block);

    /*
     * Failed to distribute block--not a fatal error (can happen for powerlaw
     * distributions). See COSM#124.
     */
    if (cfbd::dist_status::ekFAILURE == status) {
      ER_WARN("Failed to distribute block%d: fail_count=%zu",
              it->block->id().v(),
              it->fail_count);
      it->fail_count++;
      ++it;
    } else {
      /* Block location query tree already updated in success callback */

      /* Block has been distributed--remove it */
      it = m_pending_dists.erase(it);
    }
  } /* while() */

  /* unlock the arena map */
  post_block_dist_unlock(locking);
} /* disribute_single_block() */

bool base_arena_map::distribute_all_blocks(void) {
  /*
   * If we did deferred arena map initialization, some blocks might already be
   * in use in caches, so we don't distribute them.
   */
  cds::block3D_vectorno dist_blocks;
  std::copy_if(m_blocksno.begin(),
               m_blocksno.end(),
               std::back_inserter(dist_blocks),
               [&](const auto* block) { return block->is_out_of_sight(); });
  auto status = m_block_dispatcher->distribute_blocks(dist_blocks);
  ER_CHECK(cfbd::dist_status::ekSUCCESS == status,
           "Failed to distribute all blocks");

  /*
   * Once all blocks have been distributed, and (possibly) all caches have been
   * created via block consolidation, all cells that do not have anything in
   * them should be marked as empty.
   */
  for (size_t i = 0; i < xdsize(); ++i) {
    for (size_t j = 0; j < ydsize(); ++j) {
      cds::cell2D& cell = access<cads::arena_grid::kCell>(i, j);
      if (!cell.state_has_block() && !cell.state_has_cache() &&
          !cell.state_in_cache_extent() && !cell.state_in_nest_extent() &&
          !cell.state_in_block_extent()) {
        cdops::cell2D_empty_visitor op(cell.loc());
        op.visit(cell);
      }
    } /* for(j..) */
  } /* for(i..) */
  return true;

error:
  ER_FATAL_SENTINEL("Unable to perform initial block distribution");
  return false;
} /* distribute_all_blocks() */

void base_arena_map::pre_block_dist_lock(const locking& locking) {
  maybe_lock_wr(block_mtx(), !(locking & locking::ekBLOCKS_HELD));
  maybe_lock_wr(grid_mtx(), !(locking & locking::ekGRID_HELD));
} /* pre_block_dist_lock() */

void base_arena_map::post_block_dist_unlock(const locking& locking) {
  maybe_unlock_wr(grid_mtx(), !(locking & locking::ekGRID_HELD));
  maybe_unlock_wr(block_mtx(), !(locking & locking::ekBLOCKS_HELD));
} /* post_block_dist_unlock() */

const crepr::nest* base_arena_map::nest(const rtypes::type_uuid& id) const {
  auto it = m_nests->find(id);
  if (m_nests->end() != it) {
    return &(it->second);
  }
  return nullptr;
}

ds::nest_vectorro base_arena_map::nests(void) const {
  ds::nest_vectorro ret(m_nests->size());

  std::transform(m_nests->begin(),
                 m_nests->end(),
                 ret.begin(),
                 [&](const auto& pair) { return &pair.second; });
  return ret;
}

cds::block3D_vectorno base_arena_map::free_blocks(bool oos_ok) const {
  return free_blocks_calculator(oos_ok)(blocks());
} /* free_blocks() */

bool base_arena_map::placement_conflict(const crepr::sim_block3D* const block,
                                        const rmath::vector2d& loc) const {
  auto status = cspatial::conflict_checker::placement2D(this, block, loc);
  return status.x && status.y;
} /* placement_conflict() */

void base_arena_map::bloctree_update(const crepr::sim_block3D* block,
                                     const locking& locking) {
  maybe_lock_wr(block_mtx(), !(locking & locking::ekBLOCKS_HELD));

  /*
   * If the block is currently carried by a robot, it is not in the arena, so
   * don't put it in the loctree, which only contains free blocks.
   */
  if (block->is_out_of_sight()) {
    ER_INFO("Remove out of sight block%s from loctree,before_size=%zu",
            rcppsw::to_string(block->id()).c_str(),
            bloctree()->size());
    m_bloctree->remove(block);
  } else {
    ER_INFO("Update block%s in loctree (size=%zu): newloc=%s/%s",
            rcppsw::to_string(block->id()).c_str(),
            bloctree()->size(),
            rcppsw::to_string(block->ranchor2D()).c_str(),
            rcppsw::to_string(block->danchor2D()).c_str());
    m_bloctree->update(block);
  }
  ER_ASSERT(bloctree_verify(), "Bloctree failed verification");
  maybe_unlock_wr(block_mtx(), !(locking & locking::ekBLOCKS_HELD));
} /* bloctree_update() */

bool base_arena_map::bloctree_verify(void) const {
  for (auto& pair : *m_bloctree) {
    auto* block = blocks()[pair.second.v()];
    ER_CHECK(!block->is_out_of_sight(),
             "Out of sight block%s in bloctree",
             rcppsw::to_string(block->id()).c_str());
  } /* for(&pair..) */

  return true;

error:
  ER_FATAL_SENTINEL("Bloctree failed verification")
  return false;
} /* bloctree_verify() */

void base_arena_map::ordered_lock(const locking& locking) {
  maybe_lock_wr(block_mtx(), !(locking & locking::ekBLOCKS_HELD));
  maybe_lock_wr(grid_mtx(), !(locking & locking::ekGRID_HELD));
} /* ordered_lock() */

void base_arena_map::ordered_unlock(const locking& locking) {
  maybe_unlock_wr(grid_mtx(), !(locking & locking::ekGRID_HELD));
  maybe_unlock_wr(block_mtx(), !(locking & locking::ekBLOCKS_HELD));
} /* ordered_unlock() */

NS_END(arena, cosm);
