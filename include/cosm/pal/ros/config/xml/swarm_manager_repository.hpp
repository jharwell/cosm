/**
 * \file swarm_manager_repository.hpp
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
namespace cosm::pal::ros::config::xml {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class swarm_manager_repository
 * \ingroup pal ros config xml
 *
 * \brief Collection of all XML parsers and parse results common to all \ref
 * base_swarm_manager derived classes for the ROS platform.
 */
class swarm_manager_repository : public cpconfig::xml::base_swarm_manager_repository {};

} /* namespace cosm::pal::ros::config::xml */

