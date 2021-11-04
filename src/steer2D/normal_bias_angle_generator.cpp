/**
 * \file normal_bias_angle_generator.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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
#include "cosm/steer2D/normal_bias_angle_generator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, steer2D);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
rmath::radians normal_bias_angle_generator::operator()(
    const rmath::radians& last_heading,
    rmath::rng* rng) {
  /*
   * Both min and max are 3 std deviations away from the mean of 0, so it is
   * very unlikely that we will get a value outside the max deviation.
   */
  auto bias = rmath::radians(rng->gaussian(0,
                                           (2 * config()->max_delta / 6.0).v()));
  return rmath::radians(std::fmod((last_heading + bias).v(),
                                  config()->max_delta.v()));
  return bias;
} /* operator()() */

NS_END(steer2D, cosm);
