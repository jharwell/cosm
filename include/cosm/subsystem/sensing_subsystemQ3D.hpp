/**
 * \file sensing_subsystemQ3D.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * This file is part of COSM.
 *
 * COSM is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * COSM is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * COSM.  If not, see <http://www.gnu.org/licenses/
 */

#ifndef INCLUDE_COSM_SUBSYSTEM_SENSING_SUBSYSTEM_HPP_
#define INCLUDE_COSM_SUBSYSTEM_SENSING_SUBSYSTEM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/variant.hpp>
#include <map>
#include <typeindex>

#include "rcppsw/types/timestep.hpp"

#include "cosm/hal/sensors/battery_sensor.hpp"
#include "cosm/hal/sensors/diff_drive_sensor.hpp"
#include "cosm/hal/sensors/ground_sensor.hpp"
#include "cosm/hal/sensors/light_sensor.hpp"
#include "cosm/hal/sensors/position_sensor.hpp"
#include "cosm/hal/sensors/proximity_sensor.hpp"
#include "cosm/hal/sensors/wifi_sensor.hpp"
#if COSM_HAL_TARGET == HAL_TARGET_ARGOS_FOOTBOT
#include "cosm/hal/sensors/colored_blob_camera_sensor.hpp"
#endif
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
 * controllers. It uses a \ref hal::sensors::position_sensor for positioning
 * information, and manages any number of additional sensors:
 *
 * - \ref hal::sensors::proximity_sensor
 * - \ref hal::sensors::colored_blob_camera_sensor
 * - \ref hal::sensors::light_sensor
 * - \ref hal::sensors::ground_sensor
 * - \ref hal::sensors::battery_sensor
 * - \ref hal::sensors::diff_drive_sensor
 * - \ref hal::sensors::wifi_sensor
 */
class sensing_subsystemQ3D {
 public:
  using variant_type = boost::variant<
   hal::sensors::proximity_sensor,
   hal::sensors::wifi_sensor,
   hal::sensors::light_sensor,
   hal::sensors::ground_sensor,
   hal::sensors::battery_sensor,
   hal::sensors::diff_drive_sensor,
#if COSM_HAL_TARGET == HAL_TARGET_ARGOS_FOOTBOT
   hal::sensors::colored_blob_camera_sensor
#endif
   >;
  using sensor_map = std::map<std::type_index, variant_type>;

  /**
   * \brief Convenience function to create a sensor map create for the
   * specified sensor to make client code cleaner.
   */
  template <typename TSensor>
  static sensor_map::value_type map_entry_create(const TSensor& sensor) {
    return {typeid(TSensor), variant_type(sensor)};
  }

  /**
   * \param pos Position sensor.
   * \param sensors Map of handles to sensing devices, indexed by typeid.
   */
  sensing_subsystemQ3D(const hal::sensors::position_sensor& pos,
                         const sensor_map& sensors)
      : m_pos_sensor(pos), m_sensors(sensors) {}

  virtual ~sensing_subsystemQ3D(void) = default;

  const rtypes::timestep& tick(void) const { return m_tick; }

  template <typename TSensor>
  bool replace(const TSensor& sensor) {
    if (m_sensors.end() != m_sensors.find(typeid(sensor))) {
      m_sensors.erase(m_sensors.find(typeid(sensor)));
      return m_sensors.insert(map_entry_create(sensor)).second;
    }
    return false;
  }

  template <typename T>
  const T* sensor(void) const {
    return &boost::get<T>(m_sensors.find(typeid(T))->second);
  }

  template <typename T>
  T* sensor(void) {
    return &boost::get<T>(m_sensors.find(typeid(T))->second);
  }

  /**
   * \brief Get the robot's current azimuth heading; this effectively is the
   * angle of the 2D projection of the robots current position in 3D space onto
   * the XY plane.
   */
  const rmath::radians& azimuth(void) const { return m_azimuth; }

  /**
   * \brief Get the robot's current inclination heading; this effectively is the
   * angle the robots current position vector makes with the XY plane.
   */
  const rmath::radians& inclination(void) const { return m_inclination; }

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
  void update(const rtypes::timestep& t,
              const rtypes::discretize_ratio& ratio) {
    m_tick = t;
    auto reading = m_pos_sensor.reading();

    /* update 2D position info */
    m_prev_rpos2D = m_rpos2D;
    m_rpos2D = reading.position.project_on_xy();
    m_dpos2D = rmath::dvec2zvec(m_rpos2D, ratio.v());

    /* update 3D position info */
    m_prev_rpos3D = m_rpos3D;
    m_rpos3D = reading.position;
    m_dpos3D = rmath::dvec2zvec(m_rpos3D, ratio.v());
    auto sphere = m_rpos3D.to_spherical();
    m_azimuth = sphere.azimuth();
    m_inclination = sphere.inclination();
  }

  /**
   * \brief Get how far the robot has traveled in the last timestep in 2D, as
   * well as the direction/magnitude.
   */
  rmath::vector2d tick_travel2D(void) const {
    return m_rpos2D - m_prev_rpos2D;
  }

  /**
   * \brief Get how far the robot has traveled in the last timestep in 3D, as
   * well as the direction/magnitude.
   */
  rmath::vector3d tick_travel3D(void) const {
    return m_rpos3D - m_prev_rpos3D;
  }


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
  rmath::radians                m_inclination{};

  hal::sensors::position_sensor m_pos_sensor;
  sensor_map                    m_sensors;
  /* clang-format off */
};

NS_END(subsystem, cosm);

#endif /* INCLUDE_COSM_SUBSYSTEM_SENSING_SUBSYSTEM _HPP_ */
