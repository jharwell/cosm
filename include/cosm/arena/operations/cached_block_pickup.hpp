/**
 * \file cached_block_pickup.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/er/client.hpp"
#include "rcppsw/types/timestep.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "cosm/ds/operations/cell2D_op.hpp"
#include "cosm/arena/locking.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::arena::repr {
class arena_cache;
}
namespace cosm::repr {
class sim_block3D;
} /* namespace cosm::repr */

namespace cosm::arena {
class caching_arena_map;
} /* namespace cosm::arena */

namespace cosm::pal::argos {
class swarm_manager_adaptor;
} /* namespace cosm::pal */

namespace cosm::arena::operations {
namespace detail {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cached_block_pickup
 * \ingroup arena operations detail
 *
 * \brief Created whenever a robpot picks up a block from a cache.
 *
 * The cache usage penalty, if there is one, is assessed prior to this event
 * being created, at a higher level.
 */
class cached_block_pickup : public rer::client<cached_block_pickup>,
                            public cdops::cell2D_op {
 private:
  struct visit_typelist_impl {
    using inherited = cell2D_op::visit_typelist;
    using others = rmpl::typelist<caching_arena_map, crepr::sim_block3D>;
    using value = boost::mpl::joint_view<inherited::type, others::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  /**
   * \param cache The cache to pickup from in the arena.
   * \param pickup_block Handle to the block to be picked up from the cache.
   * \param sm Handle to the swarm manager.
   * \param robot_id The ID of the robot picking up the block.
   * \param t The current timestep.
   */
  cached_block_pickup(carepr::arena_cache* cache,
                      crepr::sim_block3D* pickup_block,
                      cpargos::swarm_manager_adaptor* sm,
                      const rtypes::type_uuid& robot_id,
                      const rtypes::timestep& t,
                      const locking& locking);
  ~cached_block_pickup(void) override = default;

  cached_block_pickup(const cached_block_pickup&) = delete;
  cached_block_pickup& operator=(const cached_block_pickup&) = delete;

  /**
   * \brief Perform actual cache block pickup in the arena, taking locks as
   * needed.
   */
  void visit(caching_arena_map& map);

 private:
  void visit(cds::cell2D& cell);
  void visit(cfsm::cell2D_fsm& fsm);
  void visit(carepr::arena_cache& cache);
  void visit(crepr::sim_block3D& block);


  /* clang-format off */
  const rtypes::type_uuid    mc_robot_id;
  const rtypes::timestep     mc_timestep;
  const locking              mc_locking;

  carepr::arena_cache*       m_real_cache;
  cpargos::swarm_manager_adaptor*    m_sm;

  crepr::sim_block3D*       m_pickup_block;
  /* clang-format on */
};

} /* namespace detail */

/**
 * \brief We use the precise visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using cached_block_pickup_visitor = rpvisitor::filtered_visitor<detail::cached_block_pickup>;

} /* namespace cosm::arena::operations */
