/**
 * \file arena_map.cpp
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
#include "cosm/foraging/ds/arena_map.hpp"

#include <argos3/plugins/simulator/media/led_medium.h>

#include "cosm/ds/cell2D.hpp"
#include "cosm/events/cell2D_empty.hpp"
#include "cosm/foraging/block_dist/block_manifest_processor.hpp"
#include "cosm/foraging/config/arena_map_config.hpp"
#include "cosm/foraging/events/cell2D_cache_extent.hpp"
#include "cosm/foraging/repr/arena_cache.hpp"
#include "cosm/foraging/repr/light_type_index.hpp"
#include "cosm/pal/argos_sm_adaptor.hpp"
#include "cosm/repr/base_block2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, ds);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
arena_map::arena_map(const config::arena_map_config* config)
    : ER_CLIENT_INIT("cosm.foraging.ds.arena_map"),
      decorator(config->grid.resolution,
                static_cast<uint>(config->grid.upper.x() + arena_padding()),
                static_cast<uint>(config->grid.upper.y() + arena_padding())),
      m_blockso(
          block_dist::block_manifest_processor(&config->blocks.dist.manifest)
              .create_blocks()),
      m_nest(config->nest.dims,
             config->nest.center,
             config->grid.resolution,
             cfrepr::light_type_index()[cfrepr::light_type_index::kNest]),
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
bool arena_map::initialize(pal::argos_sm_adaptor* sm, rmath::rng* rng) {
  for (auto& l : m_nest.lights()) {
    sm->AddEntity(*l);
  } /* for(&l..) */

  return m_block_dispatcher.initialize(rng);
} /* initialize() */

void arena_map::caches_add(const acache_vectoro& caches,
                           cpal::argos_sm_adaptor* sm) {
  auto& medium =
      sm->GetSimulator().GetMedium<argos::CLEDMedium>(sm->led_medium());

  /*
   * Add all lights of caches to the arena. Cache lights are added directly to
   * the LED medium, which is different than what is rendered to the screen, so
   * they actually are invisible.
   */
  for (auto& c : caches) {
    medium.AddEntity(*c->light());
  } /* for(&c..) */

  for (auto& c : caches) {
    m_cachesno.push_back(c.get());
  } /* for(&c..) */

  m_cacheso.insert(m_cacheso.end(), caches.begin(), caches.end());
  ER_INFO("Add %zu created caches, total=%zu", caches.size(), m_cacheso.size());
} /* caches_add() */

rtypes::type_uuid arena_map::robot_on_block(
    const rmath::vector2d& pos,
    const rtypes::type_uuid& ent_id) const {
  /*
   * Caches hide blocks, add even though a robot may technically be standing on
   * a block, if it is also standing in a cache, that takes priority.
   */
  auto cache_id = robot_on_cache(pos, ent_id);
  if (rtypes::constants::kNoUUID != cache_id) {
    ER_TRACE("Block hidden by cache%d", cache_id.v());
    return rtypes::constants::kNoUUID;
  }

  /*
   * If the robot actually is on the block they think they are, we can short
   * circuit what may be an expensive linear search. ent_id MIGHT be for a cache
   * we have acquired, which may cause out of bounds indexing into the blocks
   * vector, so we have to check for that.
   */
  if (ent_id != rtypes::constants::kNoUUID &&
      static_cast<size_t>(ent_id.v()) < m_blockso.size() &&
      m_blockso[ent_id.v()]->contains_point(pos)) {
    return ent_id;
  }

  /* General case: linear scan */
  for (auto& b : m_blockso) {
    if (b->contains_point(pos)) {
      return b->id();
    }
  } /* for(&b..) */
  return rtypes::constants::kNoUUID;
} /* robot_on_block() */

rtypes::type_uuid arena_map::robot_on_cache(
    const rmath::vector2d& pos,
    const rtypes::type_uuid& ent_id) const {
  /*
   * If the robot actually is on the cache they think they are, we can short
   * circuit what may be an expensive linear search. ent_id MIGHT be for a block
   * we have acquired, which may cause out of bounds indexing into the caches
   * vector, so we have to check for that.
   */
  if (ent_id != rtypes::constants::kNoUUID &&
      static_cast<size_t>(ent_id.v()) < m_cacheso.size() &&
      m_cacheso[ent_id.v()]->contains_point(pos)) {
    return ent_id;
  }

  /* General case: linear scan */
  for (auto& c : m_cacheso) {
    if (c->contains_point(pos)) {
      return c->id();
    }
  } /* for(&c..) */
  return rtypes::constants::kNoUUID;
} /* robot_on_cache() */

bool arena_map::distribute_single_block(crepr::base_block2D* block,
                                        const arena_map_locking& locking) {
  /* return TRUE because the distribution of nothing is ALWAYS successful */
  if (!m_redist_governor.dist_status()) {
    return true;
  }

  maybe_lock(cache_mtx(), !(locking & arena_map_locking::ekCACHES_HELD));
  maybe_lock(block_mtx(), !(locking & arena_map_locking::ekBLOCKS_HELD));
  maybe_lock(grid_mtx(), !(locking & arena_map_locking::ekGRID_HELD));

  /* Entities that need to be avoided during block distribution are:
   *
   * - All existing caches
   * - All existing blocks
   * - Nest
   */
  cds::const_entity_list entities;
  for (auto& cache : m_cacheso) {
    entities.push_back(cache.get());
  } /* for(&cache..) */

  std::shared_ptr<crepr::base_block2D> ent = nullptr;

  for (auto& b : m_blockso) {
    /*
     * Cannot compare via dloccmp() because the block being distributed is
     * currently out of sight, just like any other blocks currently carried by
     * robots, resulting in the wrong block being distributed.
     */
    if (!(block == b.get())) {
      entities.push_back(b.get());
    } else {
      ent = b;
    }
  } /* for(&b..) */
  entities.push_back(&m_nest);

  ER_ASSERT(
      ent->id() == block->id(),
      "ID of block to distribute != ID of block in block vector: %d != %d",
      block->id().v(),
      ent->id().v());
  bool ret = m_block_dispatcher.distribute_block(ent.get(), entities);

  maybe_unlock(grid_mtx(), !(locking & arena_map_locking::ekGRID_HELD));
  maybe_unlock(block_mtx(), !(locking & arena_map_locking::ekBLOCKS_HELD));
  maybe_unlock(cache_mtx(), !(locking & arena_map_locking::ekCACHES_HELD));

  return ret;
} /* disribute_single_block() */

void arena_map::distribute_all_blocks(void) {
  // Reset all the cells to clear old references to blocks
  decoratee().reset();

  /* distribute blocks */
  cds::const_entity_list entities;
  for (auto& cache : m_cacheso) {
    entities.push_back(cache.get());
  } /* for(&cache..) */
  entities.push_back(&m_nest);

  bool b = m_block_dispatcher.distribute_blocks(m_blocksno, entities);
  ER_ASSERT(b, "Unable to perform initial block distribution");

  /*
   * Once all blocks have been distributed, and (possibly) all caches have been
   * created via block consolidation, all cells that do not have blocks or
   * caches should be empty.
   */
  for (size_t i = 0; i < xdsize(); ++i) {
    for (size_t j = 0; j < ydsize(); ++j) {
      cds::cell2D& cell = decoratee().access<cds::arena_grid::kCell>(i, j);
      if (!cell.state_has_block() && !cell.state_has_cache() &&
          !cell.state_in_cache_extent()) {
        cevents::cell2D_empty_visitor op(cell.loc());
        op.visit(cell);
      }
    } /* for(j..) */
  }   /* for(i..) */
} /* distribute_all_blocks() */

void arena_map::cache_remove(repr::arena_cache* victim,
                             pal::argos_sm_adaptor* sm) {
  /* Remove light for cache from ARGoS */
  auto& medium =
      sm->GetSimulator().GetMedium<argos::CLEDMedium>(sm->led_medium());
  medium.RemoveEntity(*victim->light());

  ER_ASSERT(m_cachesno.size() == m_cacheso.size(),
            "Owned and access cache vectors have different sizes: %zu != %zu",
            m_cacheso.size(),
            m_cachesno.size());

  size_t before = m_cacheso.size();
  RCSW_UNUSED rtypes::type_uuid id = victim->id();

  /*
   * Lookup the victim in the cache vector, since the i-th cache is not
   * guaranteed to be in position i in the vector.
   */
  auto victim_it =
      std::find_if(m_cacheso.begin(), m_cacheso.end(), [&](const auto& c) {
        return c.get() == victim;
      });
  /*
   * Add cache to zombie vector to ensure accurate metric collection THIS
   * timestep about caches.
   */
  m_zombie_caches.push_back(*victim_it);

  /*
   * Update owned and access cache vectors, verifying that the removal worked as
   * expected.
   */
  m_cachesno.erase(std::remove(m_cachesno.begin(), m_cachesno.end(), victim));
  m_cacheso.erase(std::remove(m_cacheso.begin(), m_cacheso.end(), *victim_it));
  ER_ASSERT(m_cachesno.size() == before - 1,
            "Cache%d not removed from access vector",
            id.v());
  ER_ASSERT(m_cacheso.size() == before - 1,
            "Cache%d not removed from owned vector",
            id.v());
} /* cache_remove() */

void arena_map::cache_extent_clear(repr::arena_cache* victim) {
  auto xspan = victim->xspan();
  auto yspan = victim->yspan();

  /*
   * To reset all cells covered by the cache's extent, we simply send them a
   * CELL_EMPTY event. EXCEPT for the cell that hosted the actual cache, because
   * it is currently in the HAS_BLOCK state as part of a \ref cached_block_pickup,
   * and clearing it here will trigger an assert later.
   */
  auto xmin = static_cast<uint>(std::ceil(xspan.lb() / grid_resolution().v()));
  auto xmax = static_cast<uint>(std::ceil(xspan.ub() / grid_resolution().v()));
  auto ymin = static_cast<uint>(std::ceil(yspan.lb() / grid_resolution().v()));
  auto ymax = static_cast<uint>(std::ceil(yspan.ub() / grid_resolution().v()));

  for (uint i = xmin; i < xmax; ++i) {
    for (uint j = ymin; j < ymax; ++j) {
      rmath::vector2u c = rmath::vector2u(i, j);
      if (c != victim->dloc()) {
        ER_ASSERT(victim->contains_point(
                      rmath::uvec2dvec(c, grid_resolution().v())),
                  "Cache%d does not contain point (%u, %u) within its extent",
                  victim->id().v(),
                  i,
                  j);

        auto& cell = decoratee().access<cds::arena_grid::kCell>(i, j);
        ER_ASSERT(cell.state_in_cache_extent(),
                  "cell(%u, %u) not in CACHE_EXTENT [state=%d]",
                  i,
                  j,
                  cell.fsm().current_state());
        cevents::cell2D_empty_visitor e(c);
        e.visit(cell);
      }
    } /* for(j..) */
  }   /* for(i..) */
} /* cache_extent_clear() */

NS_END(ds, foraging, cosm);
