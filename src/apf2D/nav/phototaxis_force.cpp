/**
 * \file phototaxis_force.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/apf2D/nav/phototaxis_force.hpp"

#include <numeric>

#include "cosm/apf2D/nav/config/phototaxis_force_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::apf2D::nav {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
phototaxis_force::phototaxis_force(const config::phototaxis_force_config* config)
    : mc_max(config->max) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
rmath::vector2d
phototaxis_force::operator()(const light_sensor_readings& readings) const {
  auto accum = std::accumulate(
      readings.begin(),
      readings.end(),
      rmath::vector2d(),
      [&](const rmath::vector2d& v, const auto& r) {
        return v + rmath::vector2d(r.intensity, rmath::radians(r.angle));
      });

  return rmath::vector2d(1.0, accum.angle()) * mc_max;
} /* operator()() */

rmath::vector2d
phototaxis_force::operator()(const camera_sensor_readings& readings,
                             const rutils::color& color) const {
  auto accum = std::accumulate(readings.begin(),
                               readings.end(),
                               rmath::vector2d(),
                               [&](const rmath::vector2d& v, const auto& r) {
                                 return (r.color == color) ? v + r.vec : v;
                               });

  return rmath::vector2d(1.0, accum.angle()) * mc_max;
} /* operator()() */

} /* namespace cosm::apf2D::nav */
