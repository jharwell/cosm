/**
 * \file base_swarm_manager_repository.hpp
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
#include "rcppsw/config/xml/xml_config_repository.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, pal, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_swarm_manager_repository
 * \ingroup pal config xml
 *
 * \brief Collection of all XML parsers and parse results common to all \ref
 * base_swarm_manager derived classes.
 */
class base_swarm_manager_repository : public rconfig::xml::xml_config_repository {
 public:
  base_swarm_manager_repository(void) noexcept RCPPSW_COLD;
};

NS_END(xml, config, pal, cosm);

