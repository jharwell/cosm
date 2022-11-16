/**
 * \file normal_bias_angle_generator.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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
rmath::radians
normal_bias_angle_generator::operator()(const rmath::radians& last_heading,
                                        rmath::rng* rng) {
  /*
   * Both min and max are 3 std deviations away from the mean of 0, so it is
   * very unlikely that we will get a value outside the max deviation.
   */
  auto bias =
      rmath::radians(rng->gaussian(0, (2 * config()->max_delta / 6.0).v()));
  return rmath::radians(
      std::fmod((last_heading + bias).v(), config()->max_delta.v()));
  return bias;
} /* operator()() */

NS_END(steer2D, cosm);
