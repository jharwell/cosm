/**
 * \file robot_available_actuators.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/cosm.hpp"
#include "cosm/kin2D/governed_diff_drive.hpp"
#include "cosm/hal/actuators/diagnostic_actuator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::hal::ros::subsystem {

#if COSM_HAL_TARGET == COSM_HAL_TARGET_ROS_ETURTLEBOT3
#define COSM_HAL_ROBOT_AVAILABLE_ACTUATORS      \
  ckin2D::governed_diff_drive,                  \
    chactuators::diff_drive_actuator,           \
    chactuators::diagnostic_actuator
#endif


} /* namespace cosm::hal::ros::subsystem */
