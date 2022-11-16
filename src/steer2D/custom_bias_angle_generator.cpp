/**
 * \file custom_bias_angle_generator.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/steer2D/custom_bias_angle_generator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, steer2D);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
rmath::radians custom_bias_angle_generator::operator()(const rmath::radians&,
                                                       rmath::rng* rng) {
  return config()->angles[rng->uniform(0UL, config()->angles.size() - 1UL)];
} /* operator()() */

NS_END(steer2D, cosm);
