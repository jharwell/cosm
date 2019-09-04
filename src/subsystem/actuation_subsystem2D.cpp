/**
 * @file actuation_subsystem2D.cpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/subsystem/actuation_subsystem2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, subsystem);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
struct reset_visitor : public boost::static_visitor<void> {
  template <typename TActuator>
  void operator()(TActuator& actuator) const {
    actuator.reset();
  }
};

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
void actuation_subsystem2D::reset(void) {
  for (auto& a : m_actuators) {
    boost::apply_visitor(reset_visitor(), a.second);
  } /* for(&a..) */
} /* reset() */

NS_END(subsystem, cosm);
