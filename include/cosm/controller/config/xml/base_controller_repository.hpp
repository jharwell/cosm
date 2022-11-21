/**
 * \file base_controller_repository.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
namespace cosm::controller::config::xml {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_controller_repository
 * \ingroup controller config xml
 *
 * \brief Collection of all parameter parsers and parse results needed by
 * all controllers.
 */
class base_controller_repository
    : public virtual rconfig::xml::xml_config_repository {
 public:
  base_controller_repository(void) RCPPSW_COLD;
};

} /* namespace cosm::controller::config::xml */
