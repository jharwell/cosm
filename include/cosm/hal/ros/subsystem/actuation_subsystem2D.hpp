/**
 * \file actuation_subsystem2D.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/hal/subsystem/base_actuation_subsystem2D.hpp"
#include "cosm/hal/ros/subsystem/robot_available_actuators.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, ros, subsystem);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class actuation_subsystem2D
 * \ingroup hal ros subsystem
 *
 * \brief The actuation subsystem for any ROS robot which operates in 2D.
 */
class actuation_subsystem2D :
    public chsubsystem::base_actuation_subsystem2D<COSM_HAL_ROBOT_AVAILABLE_ACTUATORS> {
 public:
  explicit actuation_subsystem2D(actuator_map&& actuators)
      : base_actuation_subsystem2D(std::move(actuators)) {}

  COSM_HAL_ACTUATOR_ACCESSOR(robot_actuator_types,
                             ckin2D::governed_diff_drive,
                             governed_diff_drive);
  COSM_HAL_ACTUATOR_ACCESSOR(robot_actuator_types,
                             ckin2D::governed_diff_drive,
                             governed_diff_drive,
                             const);

  COSM_HAL_ACTUATOR_ACCESSOR(robot_actuator_types,
                             chactuators::diff_drive_actuator,
                             diff_drive_raw);
  COSM_HAL_ACTUATOR_ACCESSOR(robot_actuator_types,
                             chactuators::diff_drive_actuator,
                             diff_drive_raw,
                             const);

  COSM_HAL_ACTUATOR_ACCESSOR(robot_actuator_types,
                             chactuators::diagnostic_actuator,
                             diagnostics);
  COSM_HAL_ACTUATOR_ACCESSOR(robot_actuator_types,
                             chactuators::diagnostic_actuator,
                             diagnostics,
                             const);
};

NS_END(subsystem, ros, hal, cosm);
