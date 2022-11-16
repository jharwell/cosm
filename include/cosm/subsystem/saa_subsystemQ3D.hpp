/**
 * \file saa_subsystemQ3D.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <utility>

#include "cosm/hal/subsystem/actuator_map.hpp"
#include "cosm/hal/subsystem/robot_available_actuators.hpp"
#include "cosm/hal/subsystem/robot_available_sensors.hpp"
#include "cosm/hal/subsystem/sensor_map.hpp"
#include "cosm/apf2D/apf_manager.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::subsystem {
class sensing_subsystemQ3D;
class actuation_subsystem2D;
} /* namespace cosm::subsystem */

NS_START(cosm, subsystem);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class saa_subsystemQ3D
 * \ingroup subsystem
 *
 * \brief Sensing and Actuation (SAA) subsystem for the footbot robot when it is
 * operating in 3D environments (sensing in 3D, actuating in 2D). The precise
 * set of sensors/actuators abstracted away at a lower level, so that this class
 * can be used for any robot.
 */
class saa_subsystemQ3D final : public apf2D::boid,
                               public rer::client<saa_subsystemQ3D> {
 public:
  using sensing_type = csubsystem::sensing_subsystemQ3D;
  using actuation_type = csubsystem::actuation_subsystem2D;

  saa_subsystemQ3D(
      chsubsystem::sensor_variant_map<COSM_HAL_ROBOT_AVAILABLE_SENSORS>&& sensors,
      chsubsystem::actuator_variant_map<COSM_HAL_ROBOT_AVAILABLE_ACTUATORS>&&
          actuators,
      const apf2D::config::apf_manager_config* const apf_config);

  /*
   * 2D BOID interface. We report velocities, speeds, and positions that respect
   * the robot's current Z vector; that is, within the plane defined by the
   * robot's current zenith angle.
   */
  ckin::odometry odometry(void) const override;
  double max_linear_speed(void) const override RCPPSW_PURE;

  sensing_type* sensing(void) { return m_sensing.get(); }
  const sensing_type* sensing(void) const { return m_sensing.get(); }
  actuation_type* actuation(void) { return m_actuation.get(); }
  const actuation_type* actuation(void) const { return m_actuation.get(); }

  /**
   * \brief Apply the summed APF forces; change wheel speeds. Resets the
   * summed forces.
   */
  void apf2D_apply(void);

  const apf2D::apf_manager& apf2D(void) const {
    return m_apf2D;
  }
  apf2D::apf_manager& apf2D(void) { return m_apf2D; }

 private:
  /* clang-format off */
  std::unique_ptr<actuation_type> m_actuation;
  std::unique_ptr<sensing_type>   m_sensing;
  apf2D::apf_manager              m_apf2D;
  /* clang-format on */
};

NS_END(subsystem, cosm);
