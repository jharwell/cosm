/**
 * \file cell3D_empty.hpp
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

#ifndef INCLUDE_COSM_DS_OPERATIONS_CELL3D_EMPTY_HPP_
#define INCLUDE_COSM_DS_OPERATIONS_CELL3D_EMPTY_HPP_

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
 * \brief We use the picky visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using cell3D_empty_visitor_impl =
    rpvisitor::precise_visitor<cell3D_empty, cell3D_empty::visit_typelist>;

class cell3D_empty_visitor : public cell3D_empty_visitor_impl {
  using cell3D_empty_visitor_impl::cell3D_empty_visitor_impl;
};

NS_END(operations, ds, cosm);

#endif /* INCLUDE_COSM_DS_OPERATIONS_CELL3D_EMPTY_HPP_ */
