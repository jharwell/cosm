/**
 * \file free_block_pickup.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"
#include "rcppsw/types/type_uuid.hpp"
#include "rcppsw/types/timestep.hpp"

#include "cosm/ds/operations/cell2D_op.hpp"
#include "cosm/arena/locking.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena {
class base_arena_map;
namespace ds {
class arena_grid;
}
} /* namespace cosm::arena */

namespace cosm::repr {
class sim_block3D;
} /* namespace cosm::repr */

NS_START(cosm, arena, operations, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class free_block_pickup
 * \ingroup arena operations detail
 *
 * \brief Fired whenever a robot picks up a free block in the arena (i.e. one
 * that is not part of a cache), OR when a block moves within the arena.
 */
class free_block_pickup : public rer::client<free_block_pickup>,
                          public cdops::cell2D_op {
 private:
  struct visit_typelist_impl {
    using value = rmpl::typelist<carena::base_arena_map>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  static free_block_pickup by_robot(crepr::sim_block3D* block,
                                    const rtypes::type_uuid& robot_id,
                                    const rtypes::timestep& t,
                                    const locking& locking);

  static free_block_pickup by_arena(crepr::sim_block3D* block);

  ~free_block_pickup(void) override = default;

  free_block_pickup(const free_block_pickup&) = delete;
  free_block_pickup& operator=(const free_block_pickup&) = delete;

  /**
   * \brief Perform actual block pickup in the arena.
   *
   * Takes arena map grid mutex to protect grid updates. arena map block mutex
   * assumed to be held when calling this function.
   */
  void visit(base_arena_map& map);

 private:
  free_block_pickup(crepr::sim_block3D* block,
                    const rtypes::type_uuid& robot_id,
                    const rtypes::timestep& t,
                    const locking& locking);

  void visit(cads::arena_grid& grid);

  /* clang-format off */
  const rtypes::type_uuid mc_robot_id;
  const rtypes::timestep  mc_timestep;
  const locking           mc_locking;

  crepr::sim_block3D*    m_block;
  /* clang-format on */
};

/**
 * \brief We use the precise visitor in order to force compile errors if a call
 * to a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using free_block_pickup_visitor_impl =
    rpvisitor::precise_visitor<free_block_pickup,
                               free_block_pickup::visit_typelist>;

NS_END(detail);

class free_block_pickup_visitor : public detail::free_block_pickup_visitor_impl {
 public:
  using detail::free_block_pickup_visitor_impl::free_block_pickup_visitor_impl;
};


NS_END(operations, arena, cosm);

