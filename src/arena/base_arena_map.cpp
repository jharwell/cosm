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
#include "rcppsw/ds/rtree2D.hpp"

#include "cosm/arena/config/arena_map_config.hpp"
#include "cosm/arena/repr/arena_cache.hpp"
#include "cosm/arena/repr/light_type_index.hpp"
#include "cosm/ds/cell2D.hpp"
#include "cosm/ds/operations/cell2D_empty.hpp"
#include "cosm/foraging/block_dist/block3D_manifest_processor.hpp"
#include "cosm/pal/argos_sm_adaptor.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/repr/operations/nest_extent.hpp"
#include "cosm/spatial/conflict_checker.hpp"
#include "cosm/arena/free_blocks_calculator.hpp"
#include "cosm/arena/ds/loctree.hpp"
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
      m_block_dispatcher(&decoratee(),
                         config->grid.resolution,
                         &config->blocks.dist),
      m_redist_governor(&config->blocks.dist.redist_governor),
      m_bm_handler(&config->blocks.motion, m_rng),
      m_bloctree(std::make_unique<cads::loctree>()),
      m_nloctree(std::make_unique<cads::loctree>()) {
  ER_INFO("real=(%fx%f), discrete=(%zux%zu), resolution=%f",
          xrsize(),
          yrsize(),
          xdsize(),
          ydsize(),
          grid_resolution().v());

  ER_INFO("Initialize %zu nests", config->nests.nests.size());
  for (auto& nest : config->nests.nests) {
    /*
     * Trim nest size if necessary so it is an even multiple of the grid
     * resolution--many parts of the code rely on 2D spatial objects on the
     * arena floor being exact multiples of the grid size.
     */
    auto nest_dims = cspatial::dimension_checker::even_multiple(grid_resolution(),
                                                                nest.dims);
    crepr::nest inst(nest_dims,
                     nest.center,
                     config->grid.resolution,
                     carepr::light_type_index()[carepr::light_type_index::kNest]);
    /* configure nest extent */
    for (size_t i = inst.xdspan().lb(); i <= inst.xdspan().ub(); ++i) {
      for (size_t j = inst.ydspan().lb(); j <= inst.ydspan().ub(); ++j) {
        auto coord = rmath::vector2z(i, j);
        crops::nest_extent_visitor op(coord, &inst);
        op.visit(access<cds::arena_grid::kCell>(coord));
      } /* for(j..) */
    } /* for(i..) */

    /* update host cell */
    m_nests.emplace(inst.id(), inst);

    /* add to loctree */
    m_nloctree->update(&inst);
  } /* for(&nest..) */

  /* initialize non-owning block vector exposed to outside classes */
  for (auto& b : m_blockso) {
    m_blocksno.push_back(b.get());
  } /* for(&b..) */
}

base_arena_map::~base_arena_map(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool base_arena_map::initialize(pal::argos_sm_adaptor* sm) {
  /* compute block bounding box */
  auto* block = *std::max_element(m_blocksno.begin(),
                                  m_blocksno.end(),
                                  [&](const auto* b1,
                                      const auto* b2) {
                                    return b1->rdim3D() < b2->rdim3D();
                                  });
  m_block_bb = block->rdim3D();

  /* initialize nest lights */
  for (auto& pair : m_nests) {
    for (auto& l : pair.second.lights()) {
      sm->AddEntity(*l);
    } /* for(&l..) */
  }   /* for(&pair..) */

  auto precalc = block_dist_precalc(nullptr);
  bool ret = m_block_dispatcher.initialize(precalc.avoid_ents, m_block_bb, m_rng);
  ret |= distribute_all_blocks();
  return ret;
} /* initialize() */

update_status base_arena_map::update(const rtypes::timestep&) {
  size_t count = m_bm_handler.move_blocks(this);
  if (count > 0) {
    return update_status::ekBLOCK_MOTION;
  }
  return update_status::ekNONE;
} /* update() */

rtypes::type_uuid base_arena_map::robot_on_block(
    const rmath::vector2d& pos,
    const rtypes::type_uuid& ent_id) const {
  /*
   * If the robot actually is on the block they think they are, we can short
   * circuit what may be an expensive linear search. ent_id MIGHT be for a
   * non-block that a robot has acuired, which may cause out of bounds indexing
   * into the blocks vector, so we have to check for that.
   */
  if (ent_id != rtypes::constants::kNoUUID &&
      static_cast<size_t>(ent_id.v()) < m_blockso.size() &&
      m_blockso[ent_id.v()]->contains_point2D(pos)) {
    return ent_id;
  }

  /* General case: linear scan */
  for (auto& b : m_blockso) {
    if (b->contains_point2D(pos)) {
      return b->id();
    }
  } /* for(&b..) */
  return rtypes::constants::kNoUUID;
} /* robot_on_block() */

rtypes::type_uuid base_arena_map::robot_in_nest(const rmath::vector2d& pos) const {
  auto it = std::find_if(m_nests.begin(),
                         m_nests.end(),
                         [&](const auto& pair) {
                           return pair.second.contains_point2D(pos);
                         });
  if (m_nests.end() == it) {
    return rtypes::constants::kNoUUID;
  }
  return it->second.id();
} /* robot_in_nest() */

cfbd::dist_status base_arena_map::distribute_single_block(crepr::base_block3D* block,
                                                          const arena_map_locking& locking) {
  /* The distribution of nothing is ALWAYS successful */
  if (!m_redist_governor.dist_status()) {
    return cfbd::dist_status::ekSUCCESS;
  }

  /* lock the arena map */
  pre_block_dist_lock(locking);

  /* calculate the entities to avoid during distribution */
  auto precalc = block_dist_precalc(block);

  /* do the distribution */
  auto status = m_block_dispatcher.distribute_block(precalc.
                                                    dist_ent,
                                                    precalc.avoid_ents);
  ER_ASSERT(cfbd::dist_status::ekSUCCESS == status,
            "Failed to distribute block%d",
            block->id().v());

  /* Update block location query tree */
  bloctree_update(block, locking);

  /* unlock the arena map */
  post_block_dist_unlock(locking);
  return status;
} /* disribute_single_block() */

bool base_arena_map::distribute_all_blocks(void) {
  /* calculate the entities to avoid during distribution */
  auto precalc = block_dist_precalc(nullptr);

  /*
   * If we did deferred arena map initialization, some blocks might already be
   * in use in caches, so we don't distribute them.
   */
  cds::block3D_vectorno dist_blocks;
  std::copy_if(m_blocksno.begin(),
               m_blocksno.end(),
               std::back_inserter(dist_blocks),
               [&](const auto* block) { return block->is_out_of_sight();
               });

  auto status = m_block_dispatcher.distribute_blocks(dist_blocks,
                                                     precalc.avoid_ents);
  ER_CHECK(cfbd::dist_status::ekSUCCESS == status,
           "Failed to distribute all blocks");

  /*
   * Update block location query tree for all blocks. No locking is needed,
   * because this happens during initialization.
   */
  for (auto *b : blocks()) {
    bloctree_update(b, arena_map_locking::ekALL_HELD);
  } /* for(*b..) */

  /*
   * Once all blocks have been distributed, and (possibly) all caches have been
   * created via block consolidation, all cells that do not have anything in
   * them should be marked as empty.
   */
  for (size_t i = 0; i < xdsize(); ++i) {
    for (size_t j = 0; j < ydsize(); ++j) {
      cds::cell2D& cell = access<cds::arena_grid::kCell>(i, j);
      if (!cell.state_has_block() && !cell.state_has_cache() &&
          !cell.state_in_cache_extent() && !cell.state_in_nest_extent() &&
          !cell.state_in_block_extent()) {
        cdops::cell2D_empty_visitor op(cell.loc());
        op.visit(cell);
      }
    } /* for(j..) */
  }   /* for(i..) */
  return true;

error:
  ER_FATAL_SENTINEL("Unable to perform initial block distribution");
  return false;
} /* distribute_all_blocks() */

void base_arena_map::pre_block_dist_lock(const arena_map_locking& locking) {
  maybe_lock_wr(block_mtx(), !(locking & arena_map_locking::ekBLOCKS_HELD));
  maybe_lock_wr(grid_mtx(), !(locking & arena_map_locking::ekGRID_HELD));
} /* pre_block_dist_lock() */

void base_arena_map::post_block_dist_unlock(const arena_map_locking& locking) {
  maybe_unlock_wr(grid_mtx(), !(locking & arena_map_locking::ekGRID_HELD));
  maybe_unlock_wr(block_mtx(), !(locking & arena_map_locking::ekBLOCKS_HELD));
} /* post_block_dist_unlock() */

base_arena_map::block_dist_precalc_type base_arena_map::block_dist_precalc(
    const crepr::base_block3D* block) {
  /* Entities that need to be avoided during block distribution are:
   *
   * - All existing blocks
   * - Nest
   */
  block_dist_precalc_type ret;

  /*
   * If this is the initial block distribution (or distribution after reset),
   * then we can skip this, because wherever blocks are now is invalid, AND
   * after a given block is distributed, it is added to the list of entities to
   * be avoided.
   */
  if (nullptr != block) {
    for (auto& b : m_blockso) {
      /*
       * Cannot compare via dloccmp() because the block being distributed is
       * currently out of sight, just like any other blocks currently carried by
       * robots, resulting in the wrong block being distributed.
       */
      if (!(block == b.get())) {
        ret.avoid_ents.push_back(b.get());
      } else {
        ret.dist_ent = b.get();
      }
    } /* for(&b..) */
    ER_ASSERT(
        ret.dist_ent->id() == block->id(),
        "ID of block to distribute != ID of block in block vector: %d != %d",
        block->id().v(),
        ret.dist_ent->id().v());
  }

  for (auto& pair : m_nests) {
    ret.avoid_ents.push_back(&pair.second);
  } /* for(&pair..) */

  return ret;
} /* block_dist_precalc() */

ds::nest_vectorro base_arena_map::nests(void) const {
  ds::nest_vectorro ret(m_nests.size());

  std::transform(m_nests.begin(),
                 m_nests.end(),
                 ret.begin(),
                 [&](const auto& pair) { return &pair.second; });
  return ret;
}

cds::block3D_vectorno base_arena_map::free_blocks(void) const {
  return free_blocks_calculator()(blocks());
} /* free_blocks() */

bool base_arena_map::placement_conflict(const crepr::base_block3D* const block,
                                        const rmath::vector2d& loc) const {
  auto status = cspatial::conflict_checker::placement2D(this, block, loc);
  return status.x && status.y;
} /* placement_conflict() */

void base_arena_map::bloctree_update(const crepr::base_block3D* block,
                                     const arena_map_locking& locking) {
  maybe_lock_wr(block_mtx(), !(locking & arena_map_locking::ekBLOCKS_HELD));

  /*
   * If the block is currently carried by a robot, it is not in the arena, so
   * don't put it in the loctree, which only contains free blocks.
   */
  if (block->is_out_of_sight()) {
    ER_INFO("Remove out of sight block%s from loctree,size=%zu",
            rcppsw::to_string(block->id()).c_str(),
            bloctree()->size());
    m_bloctree->remove(block);
  } else {
    ER_INFO("Update block%s@%s/%s in loctree,size=%zu",
            rcppsw::to_string(block->id()).c_str(),
            rcppsw::to_string(block->ranchor2D()).c_str(),
            rcppsw::to_string(block->danchor2D()).c_str(),
            bloctree()->size());
    m_bloctree->update(block);
  }
  ER_ASSERT(bloctree_verify(), "Bloctree failed verification");
  maybe_unlock_wr(block_mtx(), !(locking & arena_map_locking::ekBLOCKS_HELD));
} /* bloctree_update() */

bool base_arena_map::bloctree_verify(void) const {
  for (auto &pair : *m_bloctree) {
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

NS_END(arena, cosm);
