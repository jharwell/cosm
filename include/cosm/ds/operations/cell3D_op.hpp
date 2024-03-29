/**
 * \file cell3D_op.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector3.hpp"
#include "rcppsw/patterns/visitor/visitor.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::ds {
class cell3D;
}
namespace cosm::fsm {
class cell3D_fsm;
}

namespace cosm::ds::operations {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cell3D_op
 * \ingroup ds operations
 *
 * \brief Non-abstract interface specifying the minimum set of classes that all
 * operations that operate on \ref cell3D objects.
 *
 * Also provided are the (x, y, z) coordinates of the cell to which the event is
 * directed. Not all derived operations may need them, but they are there.
 *
 * This class should never be instantiated, only derived from. To visit \ref
 * cell3D objects, use \ref cell3D_visitor.
 */
class cell3D_op {
 protected:
  explicit cell3D_op(const rmath::vector3z& coord) : m_coord(coord) {}

 public:
  using visit_typelist = rmpl::typelist<ds::cell3D, fsm::cell3D_fsm>;

  virtual ~cell3D_op(void) = default;

  size_t x(void) const { return m_coord.x(); }
  size_t y(void) const { return m_coord.y(); }
  size_t z(void) const { return m_coord.y(); }

  const rmath::vector3z& coord(void) const { return m_coord; }

 private:
  /* clang-format on */
  rmath::vector3z m_coord;
  /* clang-format off */
};

/**
 * \brief We use the precise visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using cell3D_op_visitor = rpvisitor::precise_visitor<cell3D_op,
                                                     cell3D_op::visit_typelist>;
} /* namespace cosm::ds::operations */

