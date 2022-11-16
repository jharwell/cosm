/**
 * \file bias_angle_generator_factory.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/steer2D/bias_angle_generator_factory.hpp"

#include "cosm/steer2D/custom_bias_angle_generator.hpp"
#include "cosm/steer2D/normal_bias_angle_generator.hpp"
#include "cosm/steer2D/uniform_bias_angle_generator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, steer2D);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
bias_angle_generator_factory::bias_angle_generator_factory(void) {
  register_type<uniform_bias_angle_generator>(kUniform);
  register_type<normal_bias_angle_generator>(kNormal);
  register_type<custom_bias_angle_generator>(kCustom);
}

NS_END(steer2D, cosm);
