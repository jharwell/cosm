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

#include "cosm/arena/ds/loctree.hpp"
#include "cosm/arena/free_blocks_calculator.hpp"
#include "cosm/arena/repr/arena_cache.hpp"
#include "cosm/arena/repr/light_type_index.hpp"
#include "cosm/pal/argos_sm_adaptor.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/spatial/conflict_checker.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, arena);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
caching_arena_map::caching_arena_map(const caconfig::arena_map_config* config,
                                     rmath::rng* rng)
    : ER_CLIENT_INIT("cosm.arena.caching_arena_map"),
      base_arena_map(config, rng),
      m_cloctree(std::make_unique<cads::loctree>()) {}

caching_arena_map::~caching_arena_map(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void caching_arena_map::caches_add(const cads::acache_vectoro& caches,
                                   cpal::argos_sm_adaptor* sm) {
  auto& medium =
      sm->GetSimulator().GetMedium<argos::CLEDMedium>(sm->led_medium());

  for (auto& c : caches) {
    /*
     * Add all lights of for the cache to the arena. Cache lights are added
     * directly to the LED medium, which is different than what is rendered to
     * the screen, so they actually are invisible.
     */
    /* sm->AddEntity(*c->light()); */
    medium.AddEntity(*c->light());

    /* Add cache to outside accessible vector */
    m_cachesno.push_back(c.get());

    /* Add cache to loctree */
    cloctree_update(c.get());
    m_cacheso.push_back(std::move(c));
  } /* for(&c..) */

  ER_INFO("Add %zu created caches, total=%zu", caches.size(), m_cacheso.size());
} /* caches_add() */

void caching_arena_map::cache_remove(repr::arena_cache* victim,
                                     pal::argos_sm_adaptor* sm) {
  ER_ASSERT(m_cachesno.size() == m_cacheso.size(),
            "Owned and access cache vectors have different sizes: %zu != %zu",
            m_cacheso.size(),
            m_cachesno.size());

  /*
   * Lookup the victim in the cache vector, since the i-th cache is not
   * guaranteed to be in position i in the vector.
   */
  auto victim_it =
      std::find_if(m_cacheso.begin(), m_cacheso.end(), [&](const auto& c) {
        return victim->dloccmp(*c);
      });
  ER_ASSERT(
      victim_it != m_cacheso.end(), "Victim cache%d not found", victim->id().v());

  /* Remove light for cache from ARGoS */
  auto& medium =
      sm->GetSimulator().GetMedium<argos::CLEDMedium>(sm->led_medium());
  medium.RemoveEntity(*(*victim_it)->light());
  /* sm->RemoveEntity(*(*victim_it)->light()); */

  size_t before = m_cacheso.size();
  RCPPSW_UNUSED rtypes::type_uuid id = victim->id();

  /*
   * Copy depleted cache to zombie vector to ensure accurate proper (1) robot
   * event processing of the cache pickp this timestep (2) metric collection
   * THIS timestep about caches.
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

  /* Remove from loctree */
  cloctree_update(victim_it->get()); /* OK because victim in zombie caches */
} /* cache_remove() */

rtypes::type_uuid
caching_arena_map::robot_on_block(const rmath::vector2d& pos,
                                  const rtypes::type_uuid& ent_id) const {
  /*
   * Caches hide blocks, add even though a robot may technically be standing on
   * a block, if it is also standing in a cache, that takes priority.
   */
  auto cache_id = robot_on_cache(pos);
  if (rtypes::constants::kNoUUID != cache_id) {
    ER_TRACE("Block hidden by cache%d", cache_id.v());
    return rtypes::constants::kNoUUID;
  }
  return base_arena_map::robot_on_block(pos, ent_id);
} /* robot_on_block() */

rtypes::type_uuid
caching_arena_map::robot_on_cache(const rmath::vector2d& pos) const {
  /*
   * We can't use the ID of the cache to index into the caches vector like we
   * can for blocks, because the ID is not guaranteed to be equal to the
   * index. So we do a linear scan, which shouldn't be that expensive because
   * there aren't ever that many caches in the arena.
   */
  for (auto& c : m_cacheso) {
    if (c->contains_point2D(pos)) {
      return c->id();
    }
  } /* for(&c..) */
  return rtypes::constants::kNoUUID;
} /* robot_on_cache() */

void caching_arena_map::pre_block_dist_lock(const arena_map_locking& locking) {
  maybe_lock_wr(cache_mtx(), !(locking & arena_map_locking::ekCACHES_HELD));
  maybe_lock_wr(block_mtx(), !(locking & arena_map_locking::ekBLOCKS_HELD));
  maybe_lock_wr(grid_mtx(), !(locking & arena_map_locking::ekGRID_HELD));
} /* pre_block_dist_lock() */

void caching_arena_map::post_block_dist_unlock(const arena_map_locking& locking) {
  maybe_unlock_wr(grid_mtx(), !(locking & arena_map_locking::ekGRID_HELD));
  maybe_unlock_wr(block_mtx(), !(locking & arena_map_locking::ekBLOCKS_HELD));
  maybe_unlock_wr(cache_mtx(), !(locking & arena_map_locking::ekCACHES_HELD));
} /* post_block_dist_unlock() */

caching_arena_map::block_dist_precalc_type
caching_arena_map::block_dist_precalc(const crepr::base_block3D* block) {
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

cds::block3D_vectorno caching_arena_map::free_blocks(void) const {
  cads::acache_vectorro rocaches;
  std::transform(caches().begin(),
                 caches().end(),
                 std::back_inserter(rocaches),
                 [&](const auto& c) { return c; });

  return free_blocks_calculator()(blocks(), rocaches);
} /* free_blocks() */

bool caching_arena_map::placement_conflict(const crepr::base_block3D* const block,
                                           const rmath::vector2d& loc) const {
  auto status = cspatial::conflict_checker::placement2D(this, block, loc);
  return status.x && status.y;
} /* placement_conflict() */

bool caching_arena_map::bloctree_verify(void) const {
  if (!base_arena_map::bloctree_verify()) {
    return false;
  }

  for (auto* b : blocks()) {
    /*
     * Any blocks that are in a cache should not be in the loctree, as they are
     * not free blocks.
     */
    auto it = std::find_if(caches().begin(), caches().end(), [b](const auto* c) {
      return c->contains_block(b);
    });
    if (caches().end() != it) {
      ER_CHECK(!bloctree()->query(b->id()),
               "Block%s@%s/%s in cache%s@%s/%s in loctree",
               rcppsw::to_string(b->id()).c_str(),
               rcppsw::to_string(b->ranchor2D()).c_str(),
               rcppsw::to_string(b->danchor2D()).c_str(),
               rcppsw::to_string((*it)->id()).c_str(),
               rcppsw::to_string((*it)->rcenter2D()).c_str(),
               rcppsw::to_string((*it)->dcenter2D()).c_str());
      continue;
    }
  } /* for(*b..) */

  return true;

error:
  return false;
} /* bloctree_verify() */

bool caching_arena_map::cloctree_verify(void) const {
  for (auto* c : caches()) {
    ER_CHECK(m_cloctree->query(c->id()),
             "Cache%s@%s/%s not in loctree",
             rcppsw::to_string(c->id()).c_str(),
             rcppsw::to_string(c->rcenter2D()).c_str(),
             rcppsw::to_string(c->dcenter2D()).c_str());
  } /* for(*b..) */
  return true;

error:
  return false;
} /* cloctree_verify() */

void caching_arena_map::bloctree_update(const crepr::base_block3D* block,
                                        const arena_map_locking& locking,
                                        const ds::acache_vectoro& created) {
  maybe_lock_wr(block_mtx(), !(locking & arena_map_locking::ekBLOCKS_HELD));

  auto it =
      std::find_if(created.begin(), created.end(), [block](const auto& cache) {
        return cache->contains_block(block);
      });

  /*
   * If the block is currently carried by a robot or in a cache don't put it in
   * the loctree, which only contains free blocks.
   */
  if (block->is_out_of_sight()) {
    ER_INFO("Remove out of sight block%s from loctree,size=%zu",
            rcppsw::to_string(block->id()).c_str(),
            bloctree()->size());
    bloctree()->remove(block);
  } else if (created.end() != it) {
    /*
     * If we do things correctly, the update function only needs to check the
     * newly created caches.
     */
    ER_INFO("Remove block%s in created cache%s@%s/%s from loctree,size=%zu",
            rcppsw::to_string(block->id()).c_str(),
            rcppsw::to_string((*it)->id()).c_str(),
            rcppsw::to_string((*it)->rcenter2D()).c_str(),
            rcppsw::to_string((*it)->dcenter2D()).c_str(),
            bloctree()->size());
    bloctree()->remove(block);
  } else {
    ER_INFO("Update block%s@%s/%s in loctree,size=%zu",
            rcppsw::to_string(block->id()).c_str(),
            rcppsw::to_string(block->ranchor2D()).c_str(),
            rcppsw::to_string(block->danchor2D()).c_str(),
            bloctree()->size());
    bloctree()->update(block);
  }
  ER_ASSERT(bloctree_verify(), "Block loctree failed verification");
  maybe_unlock_wr(block_mtx(), !(locking & arena_map_locking::ekBLOCKS_HELD));
} /* bloctree_update() */

void caching_arena_map::cloctree_update(const carepr::arena_cache* cache) {
  if (m_cloctree->query(cache->id())) {
    ER_INFO("Remove depleted cache%s from loctree,size=%zu",
            rcppsw::to_string(cache->id()).c_str(),
            cloctree()->size());
    m_cloctree->remove(cache);
  } else {
    ER_INFO("Add cache%s@%s/%s to loctree,size=%zu",
            rcppsw::to_string(cache->id()).c_str(),
            rcppsw::to_string(cache->rcenter2D()).c_str(),
            rcppsw::to_string(cache->dcenter2D()).c_str(),
            cloctree()->size());
    m_cloctree->update(cache);
  }
  ER_ASSERT(cloctree_verify(), "Cache loctree failed verification");
} /* cloctree_update() */

NS_END(arena, cosm);
