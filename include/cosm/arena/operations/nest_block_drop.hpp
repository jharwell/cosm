 /**
 * \file nest_block_drop.hpp
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

#ifndef INCLUDE_COSM_ARENA_OPERATIONS_NEST_BLOCK_DROP_HPP_
#define INCLUDE_COSM_ARENA_OPERATIONS_NEST_BLOCK_DROP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/visitor/visitor.hpp"
#include "rcppsw/types/timestep.hpp"

#include "cosm/repr/base_block2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena {
template <typename T>
class base_arena_map;
class caching_arena_map;
} /* namespace cosm::arena */

NS_START(cosm, arena, operations, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class nest_block_drop
 * \ingroup arena operations detail
 *
 * \brief Fired whenever a robot drops a block in the nest.
 */
class nest_block_drop : public rer::client<nest_block_drop> {
 private:
  struct visit_typelist_impl {
    using value = rmpl::typelist<base_arena_map<crepr::base_block2D>,
                                 caching_arena_map,
                                 crepr::base_block2D>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  /**
   * \brief Initialize a nest block drop event.
   *
   * \param robot_block Handle to the block the robot is currently carrying
   *                    (originally a clone of an arena block), which it has
   *                    given up ownership of for the drop.
   * \param t Current timestep.
   */
  nest_block_drop(std::unique_ptr<crepr::base_block2D> robot_block,
                  const rtypes::timestep& t);

  ~nest_block_drop(void) override = default;

  nest_block_drop(const nest_block_drop&) = delete;
  nest_block_drop& operator=(const nest_block_drop&) = delete;

  /**
   * \brief Perform actual nest block drop in the arena.
   *
   * Takes \ref caching_arena_map block and grid mutexes to protect block
   * re-distribution and block updates, and releases afterwards. See
   * COSM#594.
   */
  void visit(caching_arena_map& map);
  void visit(base_arena_map<crepr::base_block2D>& map);

 private:
  void visit(crepr::base_block2D& block);

  template <typename TArenaMapType>
  void do_visit(TArenaMapType& map);

  /* clang-format off */
  const rtypes::timestep               mc_timestep;

  std::unique_ptr<crepr::base_block2D> m_robot_block;
  crepr::base_block2D*                 m_arena_block{nullptr};
  /* clang-format on */
};

/**
 * \brief We use the picky visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using nest_block_drop_visitor_impl =
    rpvisitor::precise_visitor<detail::nest_block_drop,
                               detail::nest_block_drop::visit_typelist>;

NS_END(detail);

class nest_block_drop_visitor : public detail::nest_block_drop_visitor_impl {
 public:
  using detail::nest_block_drop_visitor_impl::nest_block_drop_visitor_impl;
};

NS_END(operations, arena, cosm);

#endif /* INCLUDE_COSM_ARENA_OPERATIONS_NEST_BLOCK_DROP_HPP_ */
