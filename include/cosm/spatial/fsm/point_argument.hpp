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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/patterns/fsm/event.hpp"

#include "cosm/cosm.hpp"
#include "cosm/ta/taskable_argument.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class point_argument
 * \ingroup spatial fsm
 *
 * \brief An argument that can be passed to a \ref ta::taskable function
 * which contains a 2D point and a tolerance, for use in specifying a location
 * in 2D space which should be acquired.
 */
class point_argument final : public ta::taskable_argument,
                             public rpfsm::event_data {
 public:
  point_argument(void) = default;
  point_argument(double tolerance, const rmath::vector2d& v)
      : m_tolerance(tolerance), m_point(v) {}

  ~point_argument(void) override = default;

  const rmath::vector2d& point(void) const { return m_point; }
  double tolerance(void) const { return m_tolerance; }

 private:
  /* clang-format off */
  double          m_tolerance{-1};
  rmath::vector2d m_point{};
  /* clang-format on */
};

NS_END(fsm, spatial, cosm);

