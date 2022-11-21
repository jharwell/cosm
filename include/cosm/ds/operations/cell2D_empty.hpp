/**
 * \file cell2D_empty.hpp
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
#include "rcppsw/math/vector2.hpp"

#include "cosm/ds/operations/cell2D_op.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena::ds {
class arena_grid;
} // namespace cosm::ds

namespace cosm::ds::operations {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cell2D_empty
 * \ingroup ds operations detail
 *
 * \brief Created whenever a cell needs to go from some other state to being
 * empty.
 *
 * The most common example of this is when a free block is picked up, and the
 * square that the block was on is now (probably) empty. It might not be if in
 * the same timestep a new cache is created on that same cell.
 *
 * This class should never be instantiated, only derived from. To visit \ref
 * cell2D objects, use \ref cell2D_empty_visitor.
 */
class cell2D_empty : public cell2D_op, public rer::client<cell2D_empty> {
 private:
  struct visit_typelist_impl {
    using inherited = cell2D_op::visit_typelist;
    using others = rmpl::typelist<cads::arena_grid>;
    using value = boost::mpl::joint_view<inherited::type, others::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  explicit cell2D_empty(const rmath::vector2z& coord)
      : cell2D_op(coord), ER_CLIENT_INIT("cosm.ds.operations.cell2D_empty") {}

  void visit(ds::cell2D& cell);
  void visit(fsm::cell2D_fsm& fsm);
  void visit(cads::arena_grid& grid);
};

/**
 * \brief We use the precise visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using cell2D_empty_visitor = rpvisitor::filtered_visitor<cell2D_empty>;

} /* namespace cosm::ds::operations */

