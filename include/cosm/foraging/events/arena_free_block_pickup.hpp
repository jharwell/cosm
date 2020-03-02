/**
 * \file arena_free_block_pickup.hpp
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

#ifndef INCLUDE_COSM_FORAGING_EVENTS_ARENA_FREE_BLOCK_PICKUP_HPP_
#define INCLUDE_COSM_FORAGING_EVENTS_ARENA_FREE_BLOCK_PICKUP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/er/client.hpp"
#include "rcppsw/types/timestep.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "cosm/events/cell2D_op.hpp"

#include "cosm/foraging/events/arena_block_op_visit_set.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, events, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class arena_free_block_pickup
 * \ingroup foraging events detail
 *
 * \brief Fired whenever a robot picks up a free block in the arena (i.e. one
 * that is not part of a cache).
 */
class arena_free_block_pickup : public rer::client<arena_free_block_pickup>,
                          public cevents::cell2D_op {
 private:
  struct visit_typelist_impl {
    using value = boost::mpl::joint_view<arena_block_op_visit_typelist,
                                         cevents::cell2D_op::visit_typelist>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  ~arena_free_block_pickup(void) override = default;

  arena_free_block_pickup(const arena_free_block_pickup& op) = delete;
  arena_free_block_pickup& operator=(const arena_free_block_pickup& op) = delete;

  /**
   * \brief Perform actual block pickup in the arena.
   *
   * Takes \ref arena_map grid mutex to protect block re-distribution and block
   * updates. \ref arena_map block mutex assumed to be held when calling this
   * function.
   */
  void visit(cfds::arena_map& map);
  void visit(crepr::base_block2D& block);

 protected:
  arena_free_block_pickup(const std::shared_ptr<crepr::base_block2D>& block,
                          const rtypes::type_uuid& robot_id,
                          const rtypes::timestep& t);

 private:
  /* clang-format off */
  const rtypes::timestep               mc_timestep;
  const rtypes::type_uuid              mc_robot_id;

  std::shared_ptr<crepr::base_block2D> m_block;
  /* clang-format on */
};

/**
 * \brief We use the picky visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using arena_free_block_pickup_visitor_impl =
    rpvisitor::precise_visitor<detail::arena_free_block_pickup,
                               detail::arena_free_block_pickup::visit_typelist>;

NS_END(detail);

class arena_free_block_pickup_visitor : public detail::arena_free_block_pickup_visitor_impl {
  using detail::arena_free_block_pickup_visitor_impl::arena_free_block_pickup_visitor_impl;
};

NS_END(events, foraging, cosm);

#endif /* INCLUDE_COSM_FORAGING_EVENTS_ARENA_FREE_BLOCK_PICKUP_HPP_ */
