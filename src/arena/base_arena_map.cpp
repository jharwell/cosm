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

#include <argos3/plugins/simulator/media/led_medium.h>

#include "cosm/ds/cell2D.hpp"
#include "cosm/ds/operations/cell2D_empty.hpp"
#include "cosm/foraging/block_dist/block2D_manifest_processor.hpp"
#include "cosm/foraging/block_dist/block3D_manifest_processor.hpp"
#include "cosm/arena/config/arena_map_config.hpp"
#include "cosm/arena/repr/arena_cache.hpp"
#include "cosm/arena/repr/light_type_index.hpp"
#include "cosm/pal/argos_sm_adaptor.hpp"
#include "cosm/repr/base_block2D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, arena);

template<typename T>
using manifest_processor_type = typename std::conditional<std::is_same<T,
                                                                  crepr::base_block2D>::value,
                                                          cforaging::block_dist::block2D_manifest_processor,
                                                          cforaging::block_dist::block3D_manifest_processor>::type;


/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
template<class TBlockType>
base_arena_map<TBlockType>::base_arena_map(const caconfig::arena_map_config* config)
    : ER_CLIENT_INIT("cosm.arena.base_arena_map"),
      decorator(rmath::vector2d(config->grid.dims.x() + arena_padding(),
                                config->grid.dims.y() + arena_padding()),
                config->grid.resolution),
      m_blockso(manifest_processor_type<TBlockType>(&config->blocks.dist.manifest)()),
      m_nest(config->nest.dims,
             config->nest.center,
             config->grid.resolution,
             carepr::light_type_index()[carepr::light_type_index::kNest]),
      m_block_dispatcher(&decoratee(),
                         config->grid.resolution,
                         &config->blocks.dist,
                         arena_padding()),
      m_redist_governor(&config->blocks.dist.redist_governor) {
  ER_INFO("real=(%fx%f), discrete=(%zux%zu), resolution=%f",
          xrsize(),
          yrsize(),
          xdsize(),
          ydsize(),
          grid_resolution().v());
  for (auto& b : m_blockso) {
    m_blocksno.push_back(b.get());
  } /* for(&b..) */
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
template<class TBlockType>
bool base_arena_map<TBlockType>::initialize(pal::argos_sm_adaptor* sm,
                                            rmath::rng* rng) {
  for (auto& l : m_nest.lights()) {
    sm->AddEntity(*l);
  } /* for(&l..) */

  return m_block_dispatcher.initialize(rng);
} /* initialize() */

template<class TBlockType>
rtypes::type_uuid base_arena_map<TBlockType>::robot_on_block(
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

template<class TBlockType>
bool base_arena_map<TBlockType>::distribute_single_block(TBlockType* block,
                                                         const arena_map_locking& locking) {
  /* return TRUE because the distribution of nothing is ALWAYS successful */
  if (!m_redist_governor.dist_status()) {
    return true;
  }

  /* lock the arena map */
  pre_block_dist_lock(locking);

  /* calculate the entities to avoid during distribution */
  auto precalc = block_dist_precalc(block);

  /* do the distribution */
  bool ret = m_block_dispatcher.distribute_block(precalc.dist_ent,
                                                 precalc.avoid_ents);

  /* unlock the arena map */
  post_block_dist_unlock(locking);
  return ret;
} /* disribute_single_block() */

template<class TBlockType>
void base_arena_map<TBlockType>::distribute_all_blocks(void) {
  // Reset all the cells to clear old references to blocks
  decoratee().reset();

  /* calculate the entities to avoid during distribution */
  auto precalc = block_dist_precalc(nullptr);

  bool b = m_block_dispatcher.distribute_blocks(m_blocksno, precalc.avoid_ents);
  ER_ASSERT(b, "Unable to perform initial block distribution");

  /*
   * Once all blocks have been distributed, and (possibly) all caches have been
   * created via block consolidation, all cells that do not have blocks or
   * caches should be empty.
   */
  for (size_t i = 0; i < xdsize(); ++i) {
    for (size_t j = 0; j < ydsize(); ++j) {
      cds::cell2D& cell = decoratee().template access<cds::arena_grid::kCell>(i, j);
      if (!cell.state_has_block() && !cell.state_has_cache() &&
          !cell.state_in_cache_extent()) {
        cdops::cell2D_empty_visitor op(cell.loc());
        op.visit(cell);
      }
    } /* for(j..) */
  }   /* for(i..) */
} /* distribute_all_blocks() */

template<class TBlockType>
void base_arena_map<TBlockType>::pre_block_dist_lock(const arena_map_locking& locking) {
  maybe_lock(block_mtx(), !(locking & arena_map_locking::ekBLOCKS_HELD));
  maybe_lock(grid_mtx(), !(locking & arena_map_locking::ekGRID_HELD));
} /* pre_block_dist_lock() */

template<class TBlockType>
void base_arena_map<TBlockType>::post_block_dist_unlock(
    const arena_map_locking& locking) {
  maybe_unlock(grid_mtx(), !(locking & arena_map_locking::ekGRID_HELD));
  maybe_unlock(block_mtx(), !(locking & arena_map_locking::ekBLOCKS_HELD));
} /* post_block_dist_unlock() */

template<class TBlockType>
typename base_arena_map<TBlockType>::block_dist_precalc_type base_arena_map<TBlockType>::block_dist_precalc(
    const TBlockType* block) {

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

  ret.avoid_ents.push_back(&m_nest);
  return ret;
} /* block_dist_precalc() */

/*******************************************************************************
 * Template Instantiations
 ******************************************************************************/
template class carena::base_arena_map<crepr::base_block2D>;
template class carena::base_arena_map<crepr::base_block3D>;

NS_END(arena, cosm);
