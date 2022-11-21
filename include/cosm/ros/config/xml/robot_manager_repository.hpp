/**
 * \file robot_manager_repository.hpp
 *
 * Handles parsing of all XML parameters at runtime.
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/pal/config/xml/base_swarm_manager_repository.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::ros::config::xml {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class robot_manager_repository
 * \ingroup ros config xml
 *
 * \brief Collection of all XML parsers and parse results common to all \ref
 * robot_manager_adaptor derived classes for the ROS platform.
 */
class robot_manager_repository : public cpconfig::xml::base_swarm_manager_repository {};

} /* namespace cosm::ros::config::xml */
