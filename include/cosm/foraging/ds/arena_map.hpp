/**
 * \file arena_map.hpp
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

#ifndef INCLUDE_COSM_FORAGING_DS_ARENA_MAP_HPP_
#define INCLUDE_COSM_FORAGING_DS_ARENA_MAP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <mutex>
#include <vector>
#include <string>

#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/decorator/decorator.hpp"

#include "cosm/repr/base_block2D.hpp"
#include "cosm/ds/arena_grid.hpp"
#include "cosm/foraging/ds/block_vector.hpp"
#include "cosm/foraging/ds/cache_vector.hpp"
#include "cosm/foraging/repr/arena_cache.hpp"
#include "cosm/repr/nest.hpp"
#include "cosm/foraging/block_dist/dispatcher.hpp"
#include "cosm/foraging/block_dist/redist_governor.hpp"
#include "cosm/foraging/ds/arena_map_locking.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::foraging::config {
struct arena_map_config;
}

namespace cosm::repr {
class arena_cache;
}
namespace cosm::pal {
class swarm_manager;
}
namespace cosm::ds {
class cell2D;
} /* namespace cosm::ds */

NS_START(cosm, foraging, ds);


/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class arena_map
 * \ingroup ds
 *
 * \brief Combines a 2D grid with sets of objects (blocks, caches, nests, etc.)
 * that populate the grid and move around as the state of the arena
 * changes. The idea is that the arena map should be as simple as possible,
 * providing accessors and mutators, but not more complex logic, separating the
 * data in manages from the algorithms that operate on that data.
 */
class arena_map final : public rer::client<arena_map>,
                        public rpdecorator::decorator<cds::arena_grid> {
 public:
  using grid_view = rds::base_grid2D<cds::cell2D>::grid_view;
  using const_grid_view = rds::base_grid2D<cds::cell2D>::const_grid_view;

  explicit arena_map(const config::arena_map_config* config);

  /**
   * \brief Get the list of all the blocks currently present in the arena.
   *
   * Some blocks may not be visible on the arena_map, as they are being carried
   * by robots.
   */
  block_vector& blocks(void) { return m_blocks; }
  const block_vector& blocks(void) const { return m_blocks; }

  block_vector2 blocks2(void) {
    block_vector2 ret;
    for (auto &b : m_blocks) {
      ret.push_back(b.get());
    } /* for(&b..) */
    return ret;
  }

  /**
   * \brief Get the # of blocks available in the arena.
   */
  size_t n_blocks(void) const { return m_blocks.size(); }

  /**
   * \brief Get the list of all the caches currently present in the arena and
   * active.
   */
  cache_vector& caches(void) { return m_caches; }
  const cache_vector& caches(void) const { return m_caches; }

  cache_vector2 caches2(void) {
    cache_vector2 ret;
    for (auto &c : m_caches) {
      ret.push_back(c.get());
    } /* for(&b..) */
    return ret;
  }
  /**
   * \brief Get the # of caches currently in the arena.
   */
  size_t n_caches(void) const { return m_caches.size(); }

  /**
   * \brief Add caches that have been created (by a cache manager or by robots)
   * in the arena to the current set of active caches.
   *
   * \param caches The caches to add.
   * \param sm The \ref swarm_manager.
   */
  void caches_add(const cache_vector& caches, pal::swarm_manager* sm);

  /**
   * \brief Remove a cache from the list of caches.
   *
   * \param victim The cache to remove.
   * \param sm The swarm manager (to remove light for cache).
   */
  void cache_remove(const std::shared_ptr<repr::arena_cache>& victim,
                    pal::swarm_manager* sm);

  /**
   * \brief Clear the cells that a cache covers while in the arena that are in
   * CACHE_EXTENT state, resetting them to EMPTY. Called right before deleting
   * the cache from the arena.
   *
   * \param victim The cache about to be deleted.
   *
   * \note This operation requires holding the cache and grid mutexes in
   *       multithreaded contexts.
   */
  void cache_extent_clear(const std::shared_ptr<repr::arena_cache>& victim);

  template <uint Index>
  typename cds::arena_grid::layer_value_type<Index>::value_type& access(
      const rmath::vector2u& d) {
    return decoratee().access<Index>(d);
  }
  template <uint Index>
  const typename cds::arena_grid::layer_value_type<Index>::value_type& access(
      const rmath::vector2u& d) const {
    return decoratee().access<Index>(d);
  }
  template <uint Index>
  typename cds::arena_grid::layer_value_type<Index>::value_type& access(size_t i,
                                                                   size_t j) {
    return decoratee().access<Index>(i, j);
  }
  template <uint Index>
  const typename cds::arena_grid::layer_value_type<Index>::value_type& access(
      size_t i,
      size_t j) const {
    return decoratee().access<Index>(i, j);
  }

  /**
   * \brief Distribute all blocks in the arena. Resets arena state. Should only
   * be called during (re)-initialization.
   *
   * \note This operation requires holding the block and grid mutexes in
   *       multi-threaded contetxts.
   */
  void distribute_all_blocks(void);

  /**
   * \brief Distribute a particular block in the arena, according to whatever
   * policy was specified in the .argos file.
   *
   * \param block The block to distribute.
   * \param locking Is locking needed?
   *
   * \note This operation requires holding the block and grid mutexes in
   * multithreaded contexts.
   *
   * \return \c TRUE iff distribution was successful, \c FALSE otherwise.
   */
  bool distribute_single_block(crepr::base_block2D* block,
                               const arena_map_locking& locking);

  RCPPSW_DECORATE_FUNC(xdsize, const);
  RCPPSW_DECORATE_FUNC(ydsize, const);
  RCPPSW_DECORATE_FUNC(xrsize, const);
  RCPPSW_DECORATE_FUNC(yrsize, const);

  /**
   * \brief Determine if a robot is currently on top of a block (i.e. if the
   * center of the robot has crossed over into the space occupied by the block
   * extent).
   *
   * While the robots also have their own means of checking if they are on a
   * block or not, there are some false positives, so this function is used as
   * the final arbiter when deciding whether or not to trigger a given event.
   *
   * \param pos The position of a robot.
   * \param ent_id The ID of the block the robot THINKS it is on.
   *
   * \return The ID of the block that the robot is on, or -1 if the robot is not
   * actually on a block.
   */
  rtypes::type_uuid robot_on_block(const rmath::vector2d& pos,
                                   const rtypes::type_uuid& ent_id) const;

  /**
   * \brief Determine if a robot is currently on top of a cache (i.e. if the
   * center of the robot has crossed over into the space occupied by the block
   * extent).
   *
   * While the robots also have their own means of checking if they are on a
   * cache or not, there are some false positives, so this function is used as
   * the final arbiter when deciding whether or not to trigger a cache related
   * event for a particular robot.
   *
   * \param pos The position of a robot.
   * \param ent_id The ID of the cache the robot THINKS it is on.
   *
   * \return The ID of the cache that the robot is on, or -1 if the robot is not
   * actually on a cache.
   */
  rtypes::type_uuid robot_on_cache(const rmath::vector2d& pos,
                                   const rtypes::type_uuid& ent_id) const;

  /**
   * \brief Get the subgrid for use in calculating a robot's LOS.
   *
   * \param x X coord of the center of the subgrid.
   * \param y Y coord of the center of the subgrid.
   * \param radius The radius of the subgrid.
   *
   * \return The subgrid.
   */
  grid_view subgrid(size_t x, size_t y, size_t radius) {
    return decoratee().layer<cds::arena_grid::kCell>()->subcircle(x, y, radius);
  }

  const_grid_view subgrid(size_t x, size_t y, size_t radius) const {
    return decoratee().layer<cds::arena_grid::kCell>()->subcircle(x, y, radius);
  }

  rtypes::discretize_ratio grid_resolution(void) const {
    return decoratee().resolution();
  }

  const crepr::nest& nest(void) const { return m_nest; }

  const block_dist::base_distributor* block_distributor(void) const {
    return m_block_dispatcher.distributor();
  }

  const rmath::ranged& distributable_areax(void) const {
    return m_block_dispatcher.distributable_areax();
  }

  const rmath::ranged& distributable_areay(void) const {
    return m_block_dispatcher.distributable_areay();
  }

  block_dist::redist_governor* redist_governor(void) {
    return &m_redist_governor;
  }

  /**
   * \brief The amount of padding to add to the arena map so that LOS
   * calculations when a robot is VERY close to the upper edge of the arena in x
   * or y, and the conversion to discrete coordinates/rounding would cause an
   * off-by-one out-of-bounds access are avoided.
   */
  double arena_padding(void) const { return 1.0; }

  /**
   * \brief Perform deferred initialization. This is not part the constructor so
   * that it can be verified via return code. Currently it initializes:
   *
   * - The block distributor
   * - Nest lights
   */
  bool initialize(pal::swarm_manager* loop, rmath::rng* rng);

  void maybe_lock(std::mutex* mtx, bool cond) {
    if (cond) {
      mtx->lock();
    }
  }
  void maybe_unlock(std::mutex* mtx, bool cond) {
    if (cond) {
      mtx->unlock();
    }
  }

  std::mutex* grid_mtx(void) { return decoratee().mtx(); }

  /**
   * \brief Protects simultaneous updates to the caches vector.
   */
  std::mutex* cache_mtx(void) { return &m_cache_mtx; }
  std::mutex* cache_mtx(void) const { return &m_cache_mtx; }

  /**
   * \brief Protects simultaneous updates to the blocks vector.
   */
  std::mutex* block_mtx(void) { return &m_block_mtx; }

  /**
   * \brief Clear the list of caches that have been removed this timestep.
   *
   * Having such a list is necessary in order to be able to correctly gather
   * metrics from caches that have been depleted THIS timestep about block
   * pickups. Normal cache metric collection does not encompass such zombie
   * caches.
   */
  void zombie_caches_clear(void) { m_zombie_caches.clear(); }
  const cache_vector& zombie_caches(void) const { return m_zombie_caches; }

 private:
  /* clang-format off */
  mutable std::mutex          m_cache_mtx{};
  mutable std::mutex          m_block_mtx{};

  block_vector                m_blocks;
  cache_vector                m_caches{};
  cache_vector                m_zombie_caches{};

  crepr::nest                 m_nest;
  block_dist::dispatcher      m_block_dispatcher;
  block_dist::redist_governor m_redist_governor;
  /* clang-format on */
};

NS_END(ds, foraging, cosm);

#endif /* INCLUDE_COSM_FORAGING_DS_ARENA_MAP_HPP_ */
