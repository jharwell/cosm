/**
 * \file cell2D_op.hpp
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

#ifndef INCLUDE_COSM_EVENTS_CELL2D_OP_HPP_
#define INCLUDE_COSM_EVENTS_CELL2D_OP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/patterns/visitor/visitor.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::ds {
class cell2D;
}
namespace cosm::fsm {
class cell2D_fsm;
}

NS_START(cosm, events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cell2D_op
 * \ingroup events
 *
 * \brief Non-abstract interface specifying the minimum set of classes that all
 * events that operate on \ref cell2D objects within an \ref ds::arena_grid need
 * to visit.
 *
 * Also provided are the (x, y) coordinates of the cell to which the event is
 * directed. Not all derived events may need them, but they are there.
 *
 * This class should never be instantiated, only derived from. To visit \ref
 * cell2D objects, use \ref cell2D_visitor.
 */
class cell2D_op {
 protected:
  explicit cell2D_op(const rmath::vector2u& coord) : m_coord(coord) {}

 public:
  using visit_typelist = rmpl::typelist<ds::cell2D, fsm::cell2D_fsm>;

  virtual ~cell2D_op(void) = default;

  uint x(void) const { return m_coord.x(); }
  uint y(void) const { return m_coord.y(); }

  const rmath::vector2u& coord(void) const { return m_coord; }

 private:
  /* clang-format on */
  rmath::vector2u m_coord;
  /* clang-format off */
};

/**
 * \brief We use the picky visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using cell2D_op_visitor = rpvisitor::precise_visitor<cell2D_op,
                                                     cell2D_op::visit_typelist>;
NS_END(events, cosm);

#endif /* INCLUDE_COSM_EVENTS_CELL2D_OP_HPP_ */
