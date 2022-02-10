/**
 * \file robot_manager_repository.hpp
 *
 * Handles parsing of all XML parameters at runtime.
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/pal/config/xml/base_swarm_manager_repository.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, ros, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class robot_manager_repository
 * \ingroup ros config xml
 *
 * \brief Collection of all XML parsers and parse results common to all \ref
 * base_robot_manager derived classes for the ROS platform.
 */
class robot_manager_repository : public cpconfig::xml::base_swarm_manager_repository {};

NS_END(xml, config, ros, cosm);
