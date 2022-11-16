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
#include "cosm/apf2D/nav/bias_angle_generator_factory.hpp"

#include "cosm/apf2D/nav/custom_bias_angle_generator.hpp"
#include "cosm/apf2D/nav/normal_bias_angle_generator.hpp"
#include "cosm/apf2D/nav/uniform_bias_angle_generator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D::nav {

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
bias_angle_generator_factory::bias_angle_generator_factory(void) {
  register_type<uniform_bias_angle_generator>(kUniform);
  register_type<normal_bias_angle_generator>(kNormal);
  register_type<custom_bias_angle_generator>(kCustom);
}

} /* namespace cosm::apf2D::nav */
