/**
 * \file footbot_sensing_subsystem.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_ROBOTS_FOOTBOT_FOOTBOT_SENSING_SUBSYSTEM_HPP_
#define INCLUDE_COSM_ROBOTS_FOOTBOT_FOOTBOT_SENSING_SUBSYSTEM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/subsystem/sensing_subsystem2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, robots, footbot);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class footbot_sensing_subsystem
 * \ingroup robots footbot
 *
 * \brief The sensing subsystem the footbot robot.
 */
class footbot_sensing_subsystem : public subsystem::sensing_subsystem2D {
 public:
  explicit footbot_sensing_subsystem(sensor_map& sensors)
      : sensing_subsystem2D(sensors) {}

  const hal::sensors::proximity_sensor* proximity(void) const {
    return sensor<hal::sensors::proximity_sensor>();
  }

  hal::sensors::proximity_sensor* proximity(void) {
    return sensor<hal::sensors::proximity_sensor>();
  }

  const hal::sensors::colored_blob_camera_sensor* blobs(void) const {
    return sensor<hal::sensors::colored_blob_camera_sensor>();
  }

  hal::sensors::colored_blob_camera_sensor* blobs(void) {
    return sensor<hal::sensors::colored_blob_camera_sensor>();
  }

  const hal::sensors::light_sensor* light(void) const {
    return sensor<hal::sensors::light_sensor>();
  }

  hal::sensors::light_sensor* light(void) {
    return sensor<hal::sensors::light_sensor>();
  }

  const hal::sensors::ground_sensor* ground(void) const {
    return sensor<hal::sensors::ground_sensor>();
  }

  hal::sensors::ground_sensor* ground(void) {
    return sensor<hal::sensors::ground_sensor>();
  }

  const hal::sensors::battery_sensor* battery(void) const {
    return sensor<hal::sensors::battery_sensor>();
  }

  hal::sensors::battery_sensor* battery(void) {
    return sensor<hal::sensors::battery_sensor>();
  }

  const hal::sensors::diff_drive_sensor* diff_drive(void) const {
    return sensor<hal::sensors::diff_drive_sensor>();
  }

  hal::sensors::diff_drive_sensor* diff_drive(void) {
    return sensor<hal::sensors::diff_drive_sensor>();
  }
};

NS_END(footbot, robots, cosm);

#endif /* INCLUDE_COSM_ROBOTS_FOOTBOT_FOOTBOT_SENSING_SUBSYSTEM _HPP_ */
