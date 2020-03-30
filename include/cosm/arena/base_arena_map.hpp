/**
 * \file base_arena_map.hpp
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

#ifndef INCLUDE_COSM_ARENA_BASE_ARENA_MAP_HPP_
#define INCLUDE_COSM_ARENA_BASE_ARENA_MAP_HPP_

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
#include "cosm/ds/block2D_vector.hpp"
#include "cosm/repr/nest.hpp"
#include "cosm/foraging/block_dist/dispatcher.hpp"
#include "cosm/foraging/block_dist/redist_governor.hpp"
#include "cosm/arena/arena_map_locking.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena::config {
struct arena_map_config;
}

namespace cosm::pal {
class argos_sm_adaptor;
}
namespace cosm::ds {
class cell2D;
} /* namespace cosm::ds */

NS_START(cosm, arena);


/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_arena_map
 * \ingroup ds
 *
 * \brief Combines a 2D grid with sets of objects (blocks, caches, nests, etc.)
 * that populate the grid and move around as the state of the arena
 * changes. The idea is that the arena map should be as simple as possible,
 * providing accessors and mutators, but not more complex logic, separating the
 * data in manages from the algorithms that operate on that data.
 *
 * Note that the arena map does *not* expose owning access to the objects it
 * manages. This avoids extensive use of std::shared_ptr, and greatly increases
 * efficiency with large numbers of objects.
 */
class base_arena_map : public rer::client<base_arena_map>,
                       public rpdecorator::decorator<cds::arena_grid> {
 public:
  using grid_view = rds::base_grid2D<cds::cell2D>::grid_view;
  using const_grid_view = rds::base_grid2D<cds::cell2D>::const_grid_view;

  explicit base_arena_map(const caconfig::arena_map_config* config);

  /**
   * \brief Get the list of all the blocks currently present in the arena.
   *
   * Some blocks may not be visible on the base_arena_map, as they are being
   * carried by robots.
   */
  cds::block2D_vectorno& blocks(void) { return m_blocksno; }
  const cds::block2D_vectorno& blocks(void) const { return m_blocksno; }

  /**
   * \brief Get the # of blocks available in the arena.
   */
  size_t n_blocks(void) const { return m_blockso.size(); }


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
  virtual rtypes::type_uuid robot_on_block(const rmath::vector2d& pos,
                                           const rtypes::type_uuid& ent_id) const RCSW_PURE;

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

  const cforaging::block_dist::base_distributor* block_distributor(void) const {
    return m_block_dispatcher.distributor();
  }

  const rmath::ranged& distributable_areax(void) const {
    return m_block_dispatcher.distributable_areax();
  }

  const rmath::ranged& distributable_areay(void) const {
    return m_block_dispatcher.distributable_areay();
  }

  cforaging::block_dist::redist_governor* redist_governor(void) {
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
  bool initialize(cpal::argos_sm_adaptor* sm, rmath::rng* rng);

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
   * \brief Protects simultaneous updates to the blocks vector.
   */
  std::mutex* block_mtx(void) { return &m_block_mtx; }

 protected:
  struct block_dist_precalc_type {
    cds::const_entity_list avoid_ents{};
    crepr::base_block2D* dist_ent{nullptr};
  };
  /**
   * \brief Perform necessary locking prior to (1) gathering the list of
   * entities that need to be avoided during block distribution, and (2) doing
   * block distribution.
   */
  virtual void pre_block_dist_lock(const arena_map_locking& locking);

  /**
   * \brief Perform necessary unlocking after block distribution.
   */
  virtual void post_block_dist_unlock(const arena_map_locking& locking);

  /**
   * \brief Calculate the list of entities that need to be avoided during block
   * distribution.
   *
   * \param block The block to distribute. If NULL, then this is the initial
   *              block distribution.
   */
  virtual block_dist_precalc_type block_dist_precalc(const crepr::base_block2D* block);

 private:
  /* clang-format off */
  mutable std::mutex                     m_cache_mtx{};
  mutable std::mutex                     m_block_mtx{};

  cds::block2D_vectoro                   m_blockso;
  cds::block2D_vectorno                  m_blocksno{};
  crepr::nest                            m_nest;
  cforaging::block_dist::dispatcher      m_block_dispatcher;
  cforaging::block_dist::redist_governor m_redist_governor;
  /* clang-format on */
};

NS_END(arena, cosm);

#endif /* INCLUDE_COSM_ARENA_BASE_ARENA_MAP_HPP_ */
