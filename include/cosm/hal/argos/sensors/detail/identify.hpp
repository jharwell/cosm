/**
 * \file identify.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <utility>

#include "cosm/hal/sensors/identify.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace argos {
class CCI_RangeAndBearingSensor;
class CCI_PositioningSensor;
class CCI_FootBotLightSensor;
class CCI_EPuckLightSensor;
class CCI_FootBotProximitySensor;
class CCI_EPuckProximitySensor;
class CCI_FootBotMotorGroundSensor;
class CCI_QuadRotorSpeedActuator;
class CCI_EPuckGroundSensor;
class CCI_PiPuckGroundSensor;
class CCI_DifferentialSteeringSensor;
class CCI_PiPuckDifferentialDriveSensor;
class CCI_ColoredBlobOmnidirectionalCameraSensor;
class CCI_BatterySensor;
} /* namespace argos */

namespace cosm::hal::argos::sensors::detail {

/*******************************************************************************
 * RAB Sensor Templates
 ******************************************************************************/
template<typename TSensor>
using is_rab_sensor = std::is_same<TSensor, ::argos::CCI_RangeAndBearingSensor>;

/*******************************************************************************
 * Position Sensor Templates
 ******************************************************************************/
template<typename TSensor>
using is_position_sensor = std::is_same<TSensor, ::argos::CCI_PositioningSensor>;

/*******************************************************************************
 * Light Sensor Templates
 ******************************************************************************/
template<typename TSensor>
using is_footbot_light_sensor = std::is_same<TSensor,
  ::argos::CCI_FootBotLightSensor>;

template<typename TSensor>
using is_epuck_light_sensor = std::is_same<TSensor,
  ::argos::CCI_EPuckLightSensor>;

template<typename TSensor>
using is_pipuck_light_sensor = std::is_same<TSensor, std::false_type>;

/*******************************************************************************
 * IR Sensor Templates
 ******************************************************************************/
template<typename TSensor>
using is_footbot_ir_sensor = std::is_same<TSensor,
  ::argos::CCI_FootBotProximitySensor>;

template<typename TSensor>
using is_epuck_ir_sensor = std::is_same<TSensor,
  ::argos::CCI_EPuckProximitySensor>;

template<typename TSensor>
using is_pipuck_ir_sensor = std::is_same<TSensor, std::false_type>;

/*******************************************************************************
 * Ground Sensor Templates
 ******************************************************************************/
template<typename TSensor>
using is_footbot_ground_sensor = std::is_same<TSensor,
  ::argos::CCI_FootBotMotorGroundSensor>;

template<typename TSensor>
using is_epuck_ground_sensor = std::is_same<TSensor,
  ::argos::CCI_EPuckGroundSensor>;

template<typename TSensor>
using is_pipuck_ground_sensor = std::is_same<TSensor,
  ::argos::CCI_PiPuckGroundSensor>;

/*******************************************************************************
 * Diff Drive Sensor Templates
 ******************************************************************************/
template<typename T>
using is_generic_ds_sensor = std::is_same<T,
  ::argos::CCI_DifferentialSteeringSensor>;
template<typename T>
using is_pipuck_ds_sensor = std::is_same<T,
  ::argos::CCI_PiPuckDifferentialDriveSensor>;

/*******************************************************************************
 * Quadrotor Sensor Templates
 ******************************************************************************/
template<typename T>
using is_generic_quadrotor_sensor = std::is_same<T,
  ::argos::CCI_QuadRotorSpeedActuator>;

/*******************************************************************************
 * Camera Sensor Templates
 ******************************************************************************/
template<typename TSensor>
using is_blob_camera_sensor = std::is_same<
    TSensor,
  ::argos::CCI_ColoredBlobOmnidirectionalCameraSensor>;

/*******************************************************************************
 * Battery Sensor Templates
 ******************************************************************************/
template<typename TSensor>
using is_battery_sensor = std::is_same<TSensor, ::argos::CCI_BatterySensor>;

} /* namespace cosm::hal::argos::sensors::detail */
