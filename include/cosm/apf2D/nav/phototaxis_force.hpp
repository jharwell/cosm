/**
 * \file phototaxis_force.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "cosm/hal/sensors/colored_blob_camera_sensor_reading.hpp"
#include "cosm/hal/sensors/light_sensor_reading.hpp"
#include "cosm/apf2D/boid.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D::nav {

namespace config {
struct phototaxis_force_config;
}

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * \class phototaxis_force
 * \ingroup apf2D nav
 *
 * \brief A force pushing the robot away from light sources.
 */
class phototaxis_force {
 public:
  using light_sensor_readings = std::vector<hal::sensors::light_sensor_reading>;
  using camera_sensor_readings =
      std::vector<hal::sensors::colored_blob_camera_sensor_reading>;
  explicit phototaxis_force(const config::phototaxis_force_config* config);

  rmath::vector2d operator()(const light_sensor_readings& readings) const;

  /**
   * \brief Calculate force vector to the average location of objects of the
   * specified color.
   */
  rmath::vector2d operator()(const camera_sensor_readings& readings,
                             const rutils::color& color) const;

 private:
  /* clang-format off */
  const double mc_max;
  /* clang-format on */
};

} /* namespace cosm::apf2D::nav */
