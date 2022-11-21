/**
 * \file actuation_subsystem.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/hal/subsystem/base_actuation_subsystem.hpp"

#include "cosm/hal/argos/subsystem/robot_available_actuators.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::hal::argos::subsystem {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class actuation_subsystem
 * \ingroup hal argos subsystem
 *
 * \brief The actuation subsystem for any robot on ARGoS.
 */
class actuation_subsystem:
    public chsubsystem::base_actuation_subsystem<COSM_HAL_ROBOT_AVAILABLE_ACTUATORS> {
 public:
  using actuator_map = chsubsystem::base_actuation_subsystem<
   COSM_HAL_ROBOT_AVAILABLE_ACTUATORS
   >::actuator_map;

  explicit actuation_subsystem(actuator_map&& actuators)
      : base_actuation_subsystem(std::move(actuators)) {}

  COSM_HAL_ACTUATOR_ACCESSOR(chal::actuators::diagnostic_actuator, diagnostics);
  COSM_HAL_ACTUATOR_ACCESSOR(chal::actuators::diagnostic_actuator, diagnostics, const);

#if (defined(COSM_HAL_TARGET_HAS_QUADROTOR_ACTUATOR) && \
     defined(COSM_HAL_TARGET_HAS_DIFF_DRIVE_ACTUATOR))
  #error COSM_HAL_TARGET has both quadrotor and differential drive???
#endif

#if defined(COSM_HAL_TARGET_HAS_QUADROTOR_ACTUATOR)
  COSM_HAL_ACTUATOR_ACCESSOR(chargos::actuators::quadrotor_actuator, quadrotor);
  COSM_HAL_ACTUATOR_ACCESSOR(chargos::actuators::quadrotor_actuator, quadrotor, const);

  COSM_HAL_ACTUATOR_ACCESSOR(chargos::actuators::quadrotor_actuator, locomotion);
  COSM_HAL_ACTUATOR_ACCESSOR(chargos::actuators::quadrotor_actuator, locomotion, const);
#endif

#if defined(COSM_HAL_TARGET_HAS_DIFF_DRIVE_ACTUATOR)
  COSM_HAL_ACTUATOR_ACCESSOR(ckin2D::governed_diff_drive, governed_diff_drive);
  COSM_HAL_ACTUATOR_ACCESSOR(ckin2D::governed_diff_drive, governed_diff_drive, const);

  COSM_HAL_ACTUATOR_ACCESSOR(chargos::actuators::diff_drive_actuator, locomotion);
  COSM_HAL_ACTUATOR_ACCESSOR(chargos::actuators::diff_drive_actuator, locomotion, const);
#endif
};

} /* namespace cosm::hal::argos::subsystem */
