/**
 * \file base_arena_map.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <map>
#include <memory>
#include <shared_mutex>
#include <string>
#include <vector>

#include "rcppsw/er/client.hpp"
#include "rcppsw/multithread/lockable.hpp"
#include "rcppsw/patterns/decorator/decorator.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "cosm/arena/ds/arena_grid.hpp"
#include "cosm/arena/ds/nest_vector.hpp"
#include "cosm/arena/locking.hpp"
#include "cosm/arena/update_status.hpp"
#include "cosm/ds/block3D_vector.hpp"
#include "cosm/ds/entity_vector.hpp"
#include "cosm/foraging/block_dist/base_distributor.hpp"
#include "cosm/foraging/block_dist/redist_governor.hpp"
#include "cosm/foraging/block_motion_handler.hpp"
#include "cosm/spatial/common/conflict_checker.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena::config {
struct arena_map_config;
}
namespace cosm::arena::ds {
class loctree;
} /* namespace cosm::arena::ds */

namespace cosm::pal::argos {
class swarm_manager_adaptor;
}
namespace cosm::ds {
class cell2D;
} /* namespace cosm::ds */
namespace cosm::repr {
class nest;
namespace config {
struct nests_config;
} /* namespace config */
} /* namespace cosm::repr */

namespace cosm::foraging::block_dist {
class dispatcher;
}

namespace cosm::arena {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_arena_map
 * \ingroup arena
 *
 * \brief The core data structure for swarm-arena interactions.
 *
 * Combines a 2D grid with sets of objects (blocks, caches, nests, etc.)
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
                       public rpdecorator::decorator<ds::arena_grid>,
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
  typename ds::arena_grid::layer_value_type<Index>::value_type&
  access(const rmath::vector2z& d) {
    return decoratee().template access<Index>(d);
  }
  template <size_t Index>
  const typename ds::arena_grid::layer_value_type<Index>::value_type&
  access(const rmath::vector2z& d) const {
    return decoratee().template access<Index>(d);
  }
  template <size_t Index>
  typename ds::arena_grid::layer_value_type<Index>::value_type& access(size_t i,
                                                                       size_t j) {
    return decoratee().template access<Index>(i, j);
  }
  template <size_t Index>
  const typename ds::arena_grid::layer_value_type<Index>::value_type&
  access(size_t i, size_t j) const {
    return decoratee().template access<Index>(i, j);
  }

  RCPPSW_DECORATE_DECLDEF(xdsize, const);
  RCPPSW_DECORATE_DECLDEF(ydsize, const);
  RCPPSW_DECORATE_DECLDEF(xrsize, const);
  RCPPSW_DECORATE_DECLDEF(yrsize, const);
  RCPPSW_DECORATE_DECLDEF(rdims2D, const);
  RCPPSW_DECORATE_DECLDEF(ddims2D, const);

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
   * the final arbiter when deciding whether or not to trigger a block relted
   * events.
   *
   * \param pos The position of a robot.
   * \param ent_id The ID of the block the robot THINKS it is on.
   *
   * \note The block mutex must be held when calling this function.
   *
   * \return The ID of the block that the robot is on, or -1 if the robot is not
   * actually on a block.
   */
  virtual rtypes::type_uuid robot_on_block(const rmath::vector2d& pos,
                                           const rtypes::type_uuid& ent_id) const;

  /**
   * \brief Determine if a robot is currently within the boundaries of a nest.
   *
   * While robots also have their own means of checking if they are in the nest
   * or not, there can be false positives, so this function is used as the final
   * arbiter when deciding whether or not to trigger nest related events.
   */
  rtypes::type_uuid robot_in_nest(const rmath::vector2d& pos) const RCPPSW_PURE;

  /**
   * \brief Determine if placing the specified block at the specified location
   * will cause a conflict with any entities in the arena.
   *
   * Calls \ref spatial::conflict_checker internally to do the actual checking.
   */
  virtual bool placement_conflict(const crepr::sim_block3D* block,
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
   * - Block distribution status via \ref cforaging::block_dist::redist_governor.
   *
   * Does no locking, so only safe to call in non-concurrent contexts.
   *
   * \param t The current timestep.
   * \param blocks_transported How many blocks have been cumuluatively
   *                           transported/collected so far in simulation.
   * \param convergence_status Has the currently converged, according to
   *                           configured convergence measures?
   */
  void post_step_update(const rtypes::timestep& t,
                        size_t blocks_transported,
                        bool convergence_status);

  /**
   * \brief Get the free blocks in the arena. Does no locking, so this is only
   * safe to call in non-concurrent contexts.
   *
   * \param oos_ok Are out of sight (OOS) blocks OK to include? In general they
   *               are not, BUT for powerlaw block distributions which require
   *               deferred arena map initialization, all blocks are out of
   *               sight pre-init when cluster locations are computed which
   *               causes problems with clusters overlapping with caches. See
   *               COSM#142.
   */
  virtual cds::block3D_vectorno free_blocks(bool oos_ok) const;

  /**
   * \brief Update the location index tree after the specified block has moved.
   *
   * This operation requires holding the block mutex in multithreaded contexts
   * for writing, and takes it internally if not held.
   */
  virtual void bloctree_update(const crepr::sim_block3D* block,
                               const locking& locking);

  /**
   * \brief Get the list of all the blocks currently present in the arena.
   */
  cds::block3D_vectorno& blocks(void) { return m_blocksno; }
  const cds::block3D_vectorno& blocks(void) const { return m_blocksno; }

  /**
   * \brief Distribute a particular block in the arena, according to whatever
   * policy was specified in the .argos file.
   *
   * \param block The block to distribute.
   * \param locking Currently held locks.
   *
   * \note This operation requires holding the block and grid mutexes for
   * writing, and takes them internally if they are not held.
   */
  void distribute_single_block(crepr::sim_block3D* block, const locking& locking);

  /**
   * \brief Get the bounding box large enough to contain all blocks specified in
   * the manifest.
   */
  const rmath::vector3d& block_bb(void) const { return m_block_bb; }

  /**
   * \brief Get the set of all nests in the arena.
   */
  ds::nest_vectorro nests(void) const;

  /**
   * \brief Get a specific nest in the arena by UUID.
   */
  const crepr::nest* nest(const rtypes::type_uuid& id) const RCPPSW_PURE;

  const cfbd::base_distributor* block_distributor(void) const RCPPSW_PURE;
  cfbd::base_distributor* block_distributor(void) RCPPSW_PURE;
  const rmath::rangez& distributable_cellsx(void) const RCPPSW_PURE;
  const rmath::rangez& distributable_cellsy(void) const RCPPSW_PURE;

  const cads::loctree* bloctree(void) const { return m_bloctree.get(); }
  const cads::loctree* nloctree(void) const { return m_nloctree.get(); }

  /**
   * \brief Perform deferred initialization. This is not part the constructor so
   * that it can be verified via return code. Currently it does the following:
   *
   * - Initializes the block distributor and distributes all blocks.
   * - Initializes nest lights
   * - Calculates block minimal bounding box
   * - (Possibly) initializes additional nests.
   *
   * \param sm The swarm manager.
   * \param nests Additional nests to initialize (can be NULL).
   */
  virtual bool initialize(cpargos::swarm_manager_adaptor* sm,
                          const crepr::config::nests_config* nests);

  std::shared_mutex* grid_mtx(void) { return decoratee().mtx(); }

  /**
   * \brief Protects simultaneous updates to the blocks vector.
   */
  std::shared_mutex* block_mtx(void) { return &m_block_mtx; }
  std::shared_mutex* block_mtx(void) const { return &m_block_mtx; }

  virtual void ordered_lock(const locking& locking);
  virtual void ordered_unlock(const locking& locking);

 protected:
  /**
   * \brief Perform necessary locking prior to (1) gathering the list of
   * entities that need to be avoided during block distribution, and (2) doing
   * block distribution.
   */
  virtual void pre_block_dist_lock(const locking& locking);

  /**
   * \brief Perform necessary unlocking after block distribution.
   */
  virtual void post_block_dist_unlock(const locking& locking);

  virtual bool bloctree_verify(void) const;

  ds::loctree* bloctree(void) { return m_bloctree.get(); }

  virtual cds::const_spatial_entity_vector
  initial_dist_precalc(const crepr::sim_block3D*) const {
    return {};
  };
  bool initialize_shared(cpargos::swarm_manager_adaptor* sm,
                         const crepr::config::nests_config* nests);

  /**
   * \brief Distribute all blocks in the arena. Resets arena state. Called as
   * part of \ref initialize().
   *
   * \note This operation requires holding the block and grid mutexes in
   *       multi-threaded contetxts.
   */
  bool distribute_all_blocks(void);
  cfbd::dispatcher* block_dispatcher(void) const {
    return m_block_dispatcher.get();
  }
  rmath::rng* rng(void) const { return m_rng; }

 private:
  using nest_map_type = std::map<rtypes::type_uuid, crepr::nest>;
  struct pending_dist_type {
    crepr::sim_block3D* block;
    size_t fail_count;
  };

  /**
   * \brief Perform private initialization for this class which is not shared
   * with derived classes. Currently:
   *
   * - Initialize block distributor and distribute all blocks.
   */
  bool initialize_private(void);

  /**
   * \brief Initialize the nests in \p nests. If the set of nests to initialize
   * is empty, no action is performed.
   */
  void initialize_nests(const crepr::config::nests_config* nests,
                        cpargos::swarm_manager_adaptor* sm,
                        const rtypes::discretize_ratio& resolution);

  /* clang-format off */
  mutable std::shared_mutex              m_block_mtx{};
  std::shared_mutex                      m_bloctree_mtx{};

  rmath::rng*                            m_rng;
  rmath::vector3d                        m_block_bb{};
  cds::block3D_vectoro                   m_blockso;
  cds::block3D_vectorno                  m_blocksno{};

  /**
   * For powerlaw distributions, sometimes the re-distribution of a block
   * dropped in the nest/dropped due to task abort can fail, because the \ref
   * cfbd::dispatcher can't find an available space for it (possibly because of
   * blocks previously re-distributed while the current block is being carried).
   *
   * In such cases, we add the block to the list of pending re-distributions,
   * and try to re-distribute it each time another block is distributed, in the
   * case arena conditions now are more favorable.
   */
  std::list<pending_dist_type>           m_pending_dists{};
  std::unique_ptr<cfbd::dispatcher>      m_block_dispatcher;
  cfbd::redist_governor                  m_redist_governor;
  cforaging::block_motion_handler        m_bm_handler;
  mutable std::unique_ptr<nest_map_type> m_nests;
  std::unique_ptr<cads::loctree>         m_bloctree;
  std::unique_ptr<cads::loctree>         m_nloctree;
  /* clang-format on */

 public:
  /**
   * \brief Get a handle to the block motion handler.
   */
  RCPPSW_PTRREF_DECLDEF_CONST(block_motion_handler, m_bm_handler);
  RCPPSW_PTRREF_DECLDEF(redist_governor, m_redist_governor);
};

} /* namespace cosm::arena */
