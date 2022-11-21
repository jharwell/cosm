/**
 * \file nest_extent.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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

namespace cosm::repr::operations {

namespace detail {

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

} /* namespace detail */

/**
 * \brief We use the precise visitor in order to force compile errors if a call
 * to a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using nest_extent_visitor = rpvisitor::filtered_visitor<detail::nest_extent>;

} /* namespace cosm::repr::operations */

