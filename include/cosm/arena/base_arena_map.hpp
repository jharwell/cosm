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
#include <map>
#include <memory>
#include <shared_mutex>
#include <string>
#include <vector>

#include "rcppsw/er/client.hpp"
#include "rcppsw/multithread/lockable.hpp"
#include "rcppsw/patterns/decorator/decorator.hpp"

#include "cosm/arena/arena_map_locking.hpp"
#include "cosm/arena/ds/nest_vector.hpp"
#include "cosm/arena/update_status.hpp"
#include "cosm/ds/arena_grid.hpp"
#include "cosm/ds/block3D_vector.hpp"
#include "cosm/foraging/block_dist/dispatcher.hpp"
#include "cosm/foraging/block_dist/redist_governor.hpp"
#include "cosm/foraging/block_motion_handler.hpp"
#include "cosm/repr/nest.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena::config {
struct arena_map_config;
}
namespace cosm::arena::ds {
class loctree;
} /* namespace cosm::arena::ds */

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
                       public rpdecorator::decorator<cds::arena_grid>,
                       public rmultithread::lockable {
 public:
  using grid_view = rds::base_grid2D<cds::cell2D>::grid_view;
  using const_grid_view = rds::base_grid2D<cds::cell2D>::const_grid_view;

  base_arena_map(const caconfig::arena_map_config* config, rmath::rng* rng);
  ~base_arena_map(void) override;

  /* Not move/copy constructable/assignable by default */
  base_arena_map(const base_arena_map&) = delete;
  const base_arena_map& operator=(const base_arena_map&) = delete;
  base_arena_map(base_arena_map&&) = delete;
  base_arena_map& operator=(base_arena_map&&) = delete;

  template <size_t Index>
  typename cds::arena_grid::layer_value_type<Index>::value_type&
  access(const rmath::vector2z& d) {
    return decoratee().template access<Index>(d);
  }
  template <size_t Index>
  const typename cds::arena_grid::layer_value_type<Index>::value_type&
  access(const rmath::vector2z& d) const {
    return decoratee().template access<Index>(d);
  }
  template <size_t Index>
  typename cds::arena_grid::layer_value_type<Index>::value_type&
  access(size_t i, size_t j) {
    return decoratee().template access<Index>(i, j);
  }
  template <size_t Index>
  const typename cds::arena_grid::layer_value_type<Index>::value_type&
  access(size_t i, size_t j) const {
    return decoratee().template access<Index>(i, j);
  }

  RCPPSW_DECORATE_FUNC(xdsize, const);
  RCPPSW_DECORATE_FUNC(ydsize, const);
  RCPPSW_DECORATE_FUNC(xrsize, const);
  RCPPSW_DECORATE_FUNC(yrsize, const);

  rtypes::discretize_ratio grid_resolution(void) const {
    return decoratee().resolution();
  }

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
  virtual rtypes::type_uuid
  robot_on_block(const rmath::vector2d& pos,
                 const rtypes::type_uuid& ent_id) const RCPPSW_PURE;

  /**
   * \brief Determine if a robot is currently within the boundaries of a nest.
   */
  rtypes::type_uuid robot_in_nest(const rmath::vector2d& pos) const RCPPSW_PURE;
  /**
   * \brief Determine if placing the specified block at the specified location
   * will cause a conflict with any entities in the arena.
   *
   * Calls \ref spatial::conflict_checker internally to do the actual checking.
   */

  virtual bool placement_conflict(const crepr::base_block3D* const block,
                                  const rmath::vector2d& loc) const;

  /**
   * \brief Update the arena map on the current timestep, before robot
   * controllers are run.
   *
   * Currently updates the following, in order:
   *
   * - Block motion via \ref foraging::block_mover.
   *
   * \param t The current timestep.
   */
  update_status pre_step_update(const rtypes::timestep& t);

  /**
   * \brief Update the arena map on the current timestep, after robot
   * controllers are run.
   *
   * Currently updates the following, in order:
   *
   * - Block distribution status
   */
  void post_step_update(const rtypes::timestep& t,
                        size_t blocks_transported,
                        bool convergence_status);

  /**
   * \brief Get the free blocks in the arena. Does no locking, so this is only
   * safe to call in non-concurrent contexts.
   */
  virtual cds::block3D_vectorno free_blocks(void) const;

  /**
   * \brief Update the location index tree after the specified block has moved.
   */
  void bloctree_update(const crepr::base_block3D* block,
                       const arena_map_locking& locking);

  /**
   * \brief Get the list of all the blocks currently present in the arena.
   *
   * Some blocks may not be visible on the base_arena_map, as they are being
   * carried by robots.
   */
  cds::block3D_vectorno& blocks(void) { return m_blocksno; }
  const cds::block3D_vectorno& blocks(void) const { return m_blocksno; }

  /**
   * \brief Get the # of blocks available in the arena.
   */
  size_t n_blocks(void) const { return m_blockso.size(); }

  /**
   * \brief Distribute a particular block in the arena, according to whatever
   * policy was specified in the .argos file.
   *
   * \param block The block to distribute.
   * \param locking Is locking needed?
   *
   * \note This operation requires holding the block and grid mutexes in
   * multithreaded contexts.
   */
  cfbd::dist_status distribute_single_block(crepr::base_block3D* block,
                                            const arena_map_locking& locking);

  /**
   * \brief Get the bounding box large enough to contain all blocks specified in
   * the manifest.
   */
  const rmath::vector3d& block_bb(void) const { return m_block_bb; }

  ds::nest_vectorro nests(void) const;

  RCPPSW_PURE const crepr::nest* nest(const rtypes::type_uuid& id) const {
    auto it = m_nests.find(id);
    if (m_nests.end() != it) {
      return &(it->second);
    }
    return nullptr;
  }

  const cforaging::block_dist::base_distributor* block_distributor(void) const {
    return m_block_dispatcher.distributor();
  }

  const cforaging::block_motion_handler* block_motion_handler(void) const {
    return &m_bm_handler;
  }

  const rmath::rangez& distributable_cellsx(void) const {
    return m_block_dispatcher.distributable_cellsx();
  }

  const rmath::rangez& distributable_cellsy(void) const {
    return m_block_dispatcher.distributable_cellsy();
  }

  cforaging::block_dist::redist_governor* redist_governor(void) {
    return &m_redist_governor;
  }

  const cads::loctree* bloctree(void) const { return m_bloctree.get(); }
  const cads::loctree* nloctree(void) const { return m_nloctree.get(); }

  /**
   * \brief Perform deferred initialization. This is not part the constructor so
   * that it can be verified via return code. Currently it does the following:
   *
   * - Initializes the block distributor and distributes all blocks
   * - Initializes nest lights
   * - Calculates block minimal bounding box
   */
  bool initialize(cpal::argos_sm_adaptor* sm);

  std::shared_mutex* grid_mtx(void) { return decoratee().mtx(); }

  /**
   * \brief Protects simultaneous updates to the blocks vector.
   */
  std::shared_mutex* block_mtx(void) { return &m_block_mtx; }
  std::shared_mutex* block_mtx(void) const { return &m_block_mtx; }

 protected:
  struct block_dist_precalc_type {
    cds::const_spatial_entity_vector avoid_ents{};
    crepr::base_block3D* dist_ent{ nullptr };
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
  virtual block_dist_precalc_type
  block_dist_precalc(const crepr::base_block3D* block);

  virtual bool bloctree_verify(void) const;

  ds::loctree* bloctree(void) { return m_bloctree.get(); }

 private:
  using nest_map_type = std::map<rtypes::type_uuid, crepr::nest>;

  /**
   * \brief Distribute all blocks in the arena. Resets arena state. Called as
   * part of \ref initialize().
   *
   * \note This operation requires holding the block and grid mutexes in
   *       multi-threaded contetxts.
   */
  bool distribute_all_blocks(void);

  /* clang-format off */
  mutable std::shared_mutex              m_block_mtx{};
  std::shared_mutex                      m_bloctree_mtx{};

  rmath::rng*                            m_rng;
  rmath::vector3d                        m_block_bb{};
  cds::block3D_vectoro                   m_blockso;
  cds::block3D_vectorno                  m_blocksno{};
  nest_map_type                          m_nests{};
  cforaging::block_dist::dispatcher      m_block_dispatcher;
  cforaging::block_dist::redist_governor m_redist_governor;
  cforaging::block_motion_handler        m_bm_handler;
  std::unique_ptr<cads::loctree>         m_bloctree;
  std::unique_ptr<cads::loctree>         m_nloctree;
  /* clang-format on */
};

NS_END(arena, cosm);

#endif /* INCLUDE_COSM_ARENA_BASE_ARENA_MAP_HPP_ */
