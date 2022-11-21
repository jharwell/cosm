/**
 * \file robot_dynamics_applicator_config.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"
#include "rcppsw/control/config/waveform_config.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::tv::config {

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct robot_dynamics_applicator_config
 * \ingroup tv config
 *
 * \brief Configuration for the (\ref robot_dynamics_applicator).
 */
struct robot_dynamics_applicator_config final : public rconfig::base_config {
  rct::config::waveform_config motion_throttle{};
  rct::config::waveform_config block_carry_throttle{};
};

} /* namespace cosm::tv::config */

