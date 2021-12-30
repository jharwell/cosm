/**
 * \file swarm_manager_repository.hpp
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

#ifndef INCLUDE_COSM_PAL_ROS_CONFIG_XML_SWARM_MANAGER_REPOSITORY_HPP_
#define INCLUDE_COSM_PAL_ROS_CONFIG_XML_SWARM_MANAGER_REPOSITORY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/pal/config/xml/base_swarm_manager_repository.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, pal, ros, config, xml);

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
class swarm_manager_repository : public cpconfig::xml::base_swarm_manager_repository {
 public:
  swarm_manager_repository(void) noexcept RCPPSW_COLD {}
};

NS_END(xml, config, ros, pal, cosm);

#endif /* INCLUDE_COSM_PAL_ROS_CONFIG_XML_SWARM_MANAGER_REPOSITORY_HPP_ */
