/**
 * \file point_argument.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_FSM_POINT_ARGUMENT_HPP_
#define INCLUDE_COSM_FSM_POINT_ARGUMENT_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/ta/taskable_argument.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class point_argument
 * \ingroup fsm
 *
 * \brief An argument that can be passed to a \ref rcppsw::ta::taskable function
 * which contains a 2D point and a tolerance, for use in specifying a location
 * in 2D space which should be acquired.
 */
class point_argument : public rta::taskable_argument {
 public:
  point_argument(double tolerance, const rmath::vector2d& v)
      : m_tolerance(tolerance), m_point(v) {}

  ~point_argument(void) override = default;

  const rmath::vector2d& point(void) const { return m_point; }
  double tolerance(void) const { return m_tolerance; }

 private:
  /* clang-format off */
  double         m_tolerance;
  rmath::vector2d m_point;
  /* clang-format on */
};

NS_END(fsm, cosm);

#endif /* INCLUDE_COSM_FSM_POINT_ARGUMENT_HPP_ */
