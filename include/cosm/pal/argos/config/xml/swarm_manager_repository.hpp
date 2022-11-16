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
NS_START(cosm, pal, argos, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class swarm_manager_repository
 * \ingroup pal argos config xml
 *
 * \brief Collection of all XML parsers and parse results common to all \ref
 * swarm_manager derived classes for the ARGoS platform.
 */
class swarm_manager_repository : public cpconfig::xml::base_swarm_manager_repository {
 public:
  swarm_manager_repository(void) noexcept RCPPSW_COLD;
};

NS_END(xml, config, argos, pal, cosm);
