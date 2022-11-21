/**
 * \file entities_oracle_config.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <map>
#include <string>


#include "rcppsw/config/base_config.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::oracle::config {

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct entities_oracle_config
 * \ingroup oracle config
 *
 * \brief Parameters for all-seeing oracle of entity location/size/etc.
 */
struct entities_oracle_config final : public rconfig::base_config {
  std::map<std::string, bool> types{};
};

} /* namespace cosm::oracle::config */

