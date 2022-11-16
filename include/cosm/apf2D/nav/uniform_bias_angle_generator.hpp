/**
 * \file uniform_bias_angle_generator.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/apf2D/nav/base_bias_angle_generator.hpp"
#include "cosm/apf2D/nav/config/bias_angle_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D::nav {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class uniform_bias_angle_generator
 * \ingroup apf2D nav
 *
 * \brief Generates bias angles for the \ref wander_force drawn from a uniform
 * distribution.
 */
class uniform_bias_angle_generator final
    : public base_bias_angle_generator {
 public:
  explicit uniform_bias_angle_generator(const config::bias_angle_config* config)
      : base_bias_angle_generator(config) {}

  /* Not move/copy constructable/assignable by default */
  uniform_bias_angle_generator(const uniform_bias_angle_generator&) = delete;
  uniform_bias_angle_generator&
  operator=(const uniform_bias_angle_generator&) = delete;
  uniform_bias_angle_generator(uniform_bias_angle_generator&&) = delete;
  uniform_bias_angle_generator&
  operator=(uniform_bias_angle_generator&&) = delete;

  rmath::radians operator()(const rmath::radians& last_heading,
                            rmath::rng* rng) override;
};

} /* namespace cosm::apf2D::nav */
