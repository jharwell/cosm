/**
 * \file footbot_subsystem_fwd.hpp
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

#ifndef INCLUDE_COSM_ROBOTS_FOOTBOT_FOOTBOT_SUBSYSTEM_FWD_HPP_
#define INCLUDE_COSM_ROBOTS_FOOTBOT_FOOTBOT_SUBSYSTEM_FWD_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/rcppsw.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, robots, footbot);

class footbot_saa_subsystem2D;
class footbot_saa_subsystemQ3D;
class footbot_actuation_subsystem;

template <typename TSensingSubsystem>
class footbot_sensing_subsystem;

NS_END(footbot, robots, cosm);

#endif /* INCLUDE_COSM_ROBOTS_FOOTBOT_FOOTBOT_SUBSYSTEM_FWD_HPP_ */
