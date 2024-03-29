/**
 * \file uniform_bias_angle_generator.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/apf2D/nav/uniform_bias_angle_generator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D::nav {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
rmath::radians
uniform_bias_angle_generator::operator()(const rmath::radians& last_heading,
                                         rmath::rng* rng) {
  auto bias =
      -config()->max_delta + 2 * config()->max_delta * rng->uniform(0.0, 1.0);
  return rmath::radians(
      std::fmod((last_heading + bias).v(), config()->max_delta.v()));
  return bias;
} /* operator()() */

} /* namespace cosm::apf2D::nav */
