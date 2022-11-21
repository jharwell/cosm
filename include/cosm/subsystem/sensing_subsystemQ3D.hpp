/**
 * \file sensing_subsystemQ3D.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/math/vector3.hpp"
#include "rcppsw/types/timestep.hpp"

#include "cosm/hal/subsystem/sensing_subsystemQ3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, subsystem);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class sensing_subsystemQ3D
 * \ingroup subsystem
 *
 * \brief Base sensing subsystem for all sensors used by all robot
 * controllers. It adds additional functionality to \ref
 * chsubsystem::sensing_subsystemQ3D beyond the abstraction of available robot
 * sensors.
 *
 * Note that the \ref chsensors::position_sensor MUST be in the sensor map,
 * or \ref update() will crash.
 */
class sensing_subsystemQ3D final : public chsubsystem::sensing_subsystemQ3D {
 public:
  /**
   * \param pos Position sensor.
   * \param sensors Map of handles to sensing devices, indexed by typeid.
   */
  explicit sensing_subsystemQ3D(sensor_map&& sensors)
      : chsubsystem::sensing_subsystemQ3D(std::move(sensors)) {}

  virtual ~sensing_subsystemQ3D(void) = default;

  const rtypes::timestep& tick(void) const { return m_tick; }

  /**
   * \brief Get the robot's current azimuth heading; this effectively is the
   * angle of the 2D projection of the robots current position in 3D space onto
   * the XY plane.
   */
  const rmath::radians& azimuth(void) const { return m_azimuth; }

  /**
   * \brief Get the robot's current zenith heading; this effectively is the
   * angle the robots current position vector makes with the XY plane.
   */
  const rmath::radians& zenith(void) const { return m_zenith; }

  /**
   * \brief Get the angle of the current robot's heading in 2D (same as azimuth
   * in 3D).
   */
  const rmath::radians& heading(void) const { return azimuth(); }

  /**
   * \brief Get the robot's current location in 2D real coordinates.
   */
  const rmath::vector2d& rpos2D(void) const { return m_rpos2D; }

  /**
   * \brief Get the robot's current location in 2D discrete coordinates.
   */
  const rmath::vector2z& dpos2D(void) const { return m_dpos2D; }

  /**
   * \brief Get the robot's current location in 3D real coordinates.
   */
  const rmath::vector3d& rpos3D(void) const { return m_rpos3D; }

  /**
   * \brief Get the robot's current location in 3D discrete coordinates.
   */
  const rmath::vector3z& dpos3D(void) const { return m_dpos3D; }

  /**
   * \brief Update the current time and position information for the robot.
   */
  void update(const rtypes::timestep& t, const rtypes::discretize_ratio& ratio) {
    /* update real position info */
    update(t);

    /* update discrete position info */
    auto odom = odometry()->reading();
    m_dpos2D = rmath::dvec2zvec(m_rpos2D, ratio.v());
    m_dpos3D = rmath::dvec2zvec(m_rpos3D, ratio.v());
  }

  /**
   * \brief Update the current time and position information for the robot.
   */
  void update(const rtypes::timestep& t) {
    m_tick = t;
    auto odom = odometry()->reading();

    /* update 2D position info */
    m_prev_rpos2D = m_rpos2D;
    m_rpos2D = odom.pose.position.project_on_xy();

    /* update 3D position info */
    m_prev_rpos3D = m_rpos3D;
    m_rpos3D = odom.pose.position;
    m_azimuth = odom.pose.orientation.z();
    m_zenith = odom.pose.orientation.y();
  }

  /**
   * \brief Get how far the robot has traveled in the last timestep in 2D, as
   * well as the direction/magnitude.
   */
  rmath::vector2d tick_travel2D(void) const { return m_rpos2D - m_prev_rpos2D; }

  /**
   * \brief Get how far the robot has traveled in the last timestep in 3D, as
   * well as the direction/magnitude.
   */
  rmath::vector3d tick_travel3D(void) const { return m_rpos3D - m_prev_rpos3D; }

 private:
  /* clang-format off */
  rtypes::timestep              m_tick{0};
  rmath::vector3d               m_rpos3D{};
  rmath::vector3d               m_prev_rpos3D{};
  rmath::vector3z               m_dpos3D{};

  rmath::vector2d               m_rpos2D{};
  rmath::vector2d               m_prev_rpos2D{};
  rmath::vector2z               m_dpos2D{};

  rmath::radians                m_azimuth{};
  rmath::radians                m_zenith{};
  /* clang-format off */
};

NS_END(subsystem, cosm);
