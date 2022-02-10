/**
 * \file nest_extent.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector2.hpp"

#include "cosm/ds/operations/cell2D_op.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena::ds {
class arena_grid;
} // namespace ds

namespace cosm::repr {
class nest;
} // namespace repr

NS_START(cosm, repr, operations, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class nest_extent
 * \ingroup repr operations
 *
 * \brief Created whenever a cell needs to be marked as part of a nest's extent
 * (duh). Cells so marked will not change state, as the nest does not need to do
 * anything with their state.
 */
class nest_extent : public cdops::cell2D_op {
 private:
  struct visit_typelist_impl {
    using inherited = cell2D_op::visit_typelist;
    using others = rmpl::typelist<cads::arena_grid>;
    using value = rmpl::typelist<cads::arena_grid, cds::cell2D>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  nest_extent(const rmath::vector2z& coord,
              crepr::nest* nest);
  nest_extent& operator=(const nest_extent&) = delete;
  nest_extent(const nest_extent&) = delete;

  void visit(cds::cell2D& cell);
  void visit(cfsm::cell2D_fsm& fsm);
  void visit(cads::arena_grid& grid);

 private:
  /* clang-format off */
  crepr::nest* m_nest;
  /* clang-format on */
};

NS_END(detail);

/**
 * \brief We use the precise visitor in order to force compile errors if a call
 * to a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using nest_extent_visitor = rpvisitor::filtered_visitor<detail::nest_extent>;

NS_END(operations, repr, cosm);

