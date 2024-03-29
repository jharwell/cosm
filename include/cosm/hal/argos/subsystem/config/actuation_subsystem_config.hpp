/**
 * \file actuation_subsystem_config.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"
#include "cosm/kin2D/config/diff_drive_config.hpp"
#include "cosm/apf2D/config/apf_manager_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::hal::argos::subsystem::config {

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/

/**
 * \struct actuation_subsystem_config
 * \ingroup hal argos subsystem config
 *
 * \brief Configuration for the actuation subsystem for wheeled robots that
 * operate in two dimensions.
 */
struct actuation_subsystem_config final : public rconfig::base_config {
  ckin2D::config::diff_drive_config  diff_drive{};
  capf2D::config::apf_manager_config apf_manager{};
};

} /* namespace cosm::hal::argos::subsystem::config */
