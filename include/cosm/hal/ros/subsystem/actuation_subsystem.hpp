/**
 * \file actuation_subsystem.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/hal/subsystem/base_actuation_subsystem.hpp"
#include "cosm/hal/ros/subsystem/robot_available_actuators.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::hal::ros::subsystem {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class actuation_subsystem
 * \ingroup hal ros subsystem
 *
 * \brief The actuation subsystem for any ROS robot.
 */
class actuation_subsystem :
    public chsubsystem::base_actuation_subsystem<COSM_HAL_ROBOT_AVAILABLE_ACTUATORS> {
 public:
  explicit actuation_subsystem(actuator_map&& actuators)
      : base_actuation_subsystem(std::move(actuators)) {}

  COSM_HAL_ACTUATOR_ACCESSOR(ckin2D::governed_diff_drive, governed_diff_drive);
  COSM_HAL_ACTUATOR_ACCESSOR(ckin2D::governed_diff_drive, governed_diff_drive, const);

  COSM_HAL_ACTUATOR_ACCESSOR(chactuators::diff_drive_actuator, diff_drive_raw);
  COSM_HAL_ACTUATOR_ACCESSOR(chactuators::diff_drive_actuator, diff_drive_raw, const);
  COSM_HAL_ACTUATOR_ACCESSOR(chactuators::diagnostic_actuator, diagnostics);
  COSM_HAL_ACTUATOR_ACCESSOR(chactuators::diagnostic_actuator, diagnostics, const);

  COSM_HAL_ACTUATOR_ACCESSOR(chactuators::diff_drive_actuator, locomotion);
  COSM_HAL_ACTUATOR_ACCESSOR(chactuators::diff_drive_actuator, locomotion, const);
};

} /* namespace cosm::hal::ros::subsystem */
