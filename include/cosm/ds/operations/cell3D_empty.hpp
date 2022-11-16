/**
 * \file cell3D_empty.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"

#include "cosm/ds/operations/cell3D_op.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, ds, operations);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cell3D_empty
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
 * cell3D objects, use \ref cell3D_empty_visitor.
 */
class cell3D_empty : public cell3D_op, public rer::client<cell3D_empty> {
 private:
  struct visit_typelist_impl {
    using value = cell3D_op::visit_typelist;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  explicit cell3D_empty(const rmath::vector3z& coord)
      : cell3D_op(coord), ER_CLIENT_INIT("cosm.ds.operations.cell3D_empty") {}

  void visit(ds::cell3D& cell);
  void visit(fsm::cell3D_fsm& fsm);
};

/**
 * \brief We use the precise visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using cell3D_empty_visitor = rpvisitor::filtered_visitor<cell3D_empty>;

NS_END(operations, ds, cosm);

