/**
 * \file cached_block_pickup.hpp
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

#ifndef INCLUDE_COSM_ARENA_OPERATIONS_CACHED_BLOCK_PICKUP_HPP_
#define INCLUDE_COSM_ARENA_OPERATIONS_CACHED_BLOCK_PICKUP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/er/client.hpp"
#include "rcppsw/types/timestep.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "cosm/ds/operations/cell2D_op.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena::repr {
class arena_cache;
}
namespace cosm::repr {
class base_block3D;
} /* namespace cosm::repr */

namespace cosm::arena {
class caching_arena_map;
} /* namespace cosm::arena */

namespace cosm::pal {
class argos_sm_adaptor;
} /* namespace cosm::pal */

NS_START(cosm, arena, operations, detail);

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
    using others = rmpl::typelist<caching_arena_map, crepr::base_block3D>;
    using value = boost::mpl::joint_view<inherited::type, others::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  /**
   * \param cache The cache to pickup from in the arena.
   * \param sm Handle to the swarm manager.
   * \param robot_id The ID of the robot picking up the block.
   * \param t The current timestep.
   */
  cached_block_pickup(carepr::arena_cache* cache,
                      cpal::argos_sm_adaptor* sm,
                      const rtypes::type_uuid& robot_id,
                      const rtypes::timestep& t);
  ~cached_block_pickup(void) override = default;

  cached_block_pickup(const cached_block_pickup&) = delete;
  cached_block_pickup& operator=(const cached_block_pickup&) = delete;

  /**
   * \brief Perform actual cache block pickup in the arena.
   *
   * Assumes caller is holding \ref caching_arena_map cache mutex. Takes \ref
   * caching_arena_map block mutex, and then releases it after cache updates.
   */
  void visit(caching_arena_map& map);

 private:
  void visit(cds::cell2D& cell);
  void visit(cfsm::cell2D_fsm& fsm);
  void visit(crepr::base_block3D& block);
  void visit(carepr::arena_cache& cache);

  /* clang-format off */
  const rtypes::type_uuid mc_robot_id;
  const rtypes::timestep  mc_timestep;

  carepr::arena_cache*    m_real_cache;

  cpal::argos_sm_adaptor* m_sm;

  /**
   * \brief The block that will be picked up by the robot.
   */
  crepr::base_block3D*    m_pickup_block{nullptr};

  /**
   * \brief The block that is left over when a cache devolves into a single
   * block, that needs to be sent to the cell that the cache used to live on.
   */
  crepr::base_block3D*    m_orphan_block{nullptr};
  /* clang-format on */
};

/**
 * \brief We use the picky visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using cached_block_pickup_visitor_impl =
    rpvisitor::precise_visitor<detail::cached_block_pickup,
                               detail::cached_block_pickup::visit_typelist>;

NS_END(detail);

class cached_block_pickup_visitor
    : public detail::cached_block_pickup_visitor_impl {
 public:
  using detail::cached_block_pickup_visitor_impl::cached_block_pickup_visitor_impl;
};

NS_END(operations, arena, cosm);

#endif /* INCLUDE_COSM_ARENA_OPERATIONS_CACHED_BLOCK_PICKUP_HPP_ */
