/**
 * \file pal.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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
#include <string>

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Macros
 ******************************************************************************/
#if defined(COSM_ENABLE_PAL_TARGET_ARGOS)
/**
 * \brief The configuration definition to compile for the ARGoS simulator.
 */
#define COSM_PAL_TARGET_ARGOS 1

#elif defined(COSM_ENABLE_PAL_TARGET_ROS)

/**
 * \brief The configuration definition to compile for deploying ROS to a REAL
 * robot.
 */
#define COSM_PAL_TARGET_ROS 2

#endif

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, pal);

/**
 * \brief The name of the type robots within the swarm from the POV of
 * ARGoS/ROS.
 */
extern const std::string kRobotType;

/**
 * \brief The prefix that all robot unique IDs have within ARGoS/ROS.
 */
extern const std::string kRobotNamePrefix;

/**
 * \brief The unique name attached to the controller of the desired type in the
 * XML input file.
 */
extern const std::string kControllerXMLId;

#if defined(COSM_PAL_TARGET_ROS)

/**
 * \brief A hash of the current git HEAD for use in version matching messages
 * using message_traits.
 */
extern const std::string kMsgTraitsMD5;

#endif

NS_END(pal, cosm)
