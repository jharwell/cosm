/**
 * \file pal.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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
namespace cosm::pal {

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

} /* namespace cosm::pal */
