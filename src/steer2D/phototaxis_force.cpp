/**
 * \file phototaxis_force.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/steer2D/phototaxis_force.hpp"

#include <numeric>

#include "cosm/steer2D/config/phototaxis_force_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, steer2D);

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

NS_END(steer2D, cosm);
