/**
 * \file caching_arena_map.cpp
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
#include "cosm/arena/caching_arena_map.hpp"

#include <argos3/plugins/simulator/media/led_medium.h>

#include "rcppsw/utils/maskable_enum.hpp"

#include "cosm/arena/repr/arena_cache.hpp"
#include "cosm/arena/repr/light_type_index.hpp"
#include "cosm/pal/argos_sm_adaptor.hpp"
#include "cosm/repr/base_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, arena);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
caching_arena_map::caching_arena_map(const caconfig::arena_map_config* config)
    : ER_CLIENT_INIT("cosm.arena.caching_arena_map"), base_arena_map(config) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void caching_arena_map::caches_add(const cads::acache_vectoro& caches,
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

rtypes::type_uuid caching_arena_map::robot_on_block(
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
  return base_arena_map::robot_on_block(pos, ent_id);
} /* robot_on_block() */

rtypes::type_uuid caching_arena_map::robot_on_cache(
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
      m_cacheso[ent_id.v()]->contains_point2D(pos)) {
    return ent_id;
  }

  /* General case: linear scan */
  for (auto& c : m_cacheso) {
    if (c->contains_point2D(pos)) {
      return c->id();
    }
  } /* for(&c..) */
  return rtypes::constants::kNoUUID;
} /* robot_on_cache() */

void caching_arena_map::cache_remove(repr::arena_cache* victim,
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
  m_zombie_caches.push_back(victim_it->get());

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

void caching_arena_map::pre_block_dist_lock(const arena_map_locking& locking) {
  maybe_lock(cache_mtx(), !(locking & arena_map_locking::ekCACHES_HELD));
  maybe_lock(block_mtx(), !(locking & arena_map_locking::ekBLOCKS_HELD));
  maybe_lock(grid_mtx(), !(locking & arena_map_locking::ekGRID_HELD));
} /* pre_block_dist_lock() */

void caching_arena_map::post_block_dist_unlock(const arena_map_locking& locking) {
  maybe_unlock(grid_mtx(), !(locking & arena_map_locking::ekGRID_HELD));
  maybe_unlock(block_mtx(), !(locking & arena_map_locking::ekBLOCKS_HELD));
  maybe_unlock(cache_mtx(), !(locking & arena_map_locking::ekCACHES_HELD));
} /* post_block_dist_unlock() */

caching_arena_map::block_dist_precalc_type caching_arena_map::block_dist_precalc(
    const crepr::base_block3D* block) {
  auto ret = base_arena_map::block_dist_precalc(block);

  /*
   * Additional entities that need to be avoided during block distribution are:
   *
   * - All existing caches
   */
  cds::const_entity_vector entities;
  for (auto& cache : m_cacheso) {
    ret.avoid_ents.push_back(cache.get());
  } /* for(&cache..) */
  return ret;
} /* block_dist_precalc() */

NS_END(arena, cosm);
