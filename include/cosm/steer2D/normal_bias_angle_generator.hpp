/**
 * \file normal_bias_angle_generator.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/steer2D/base_bias_angle_generator.hpp"
#include "cosm/steer2D/config/bias_angle_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, steer2D);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class normal_bias_angle_generator
 * \ingroup steer2D
 *
 * \brief Generates bias angles for the \ref wander_force drawn from a normal
 * distribution.
 */
class normal_bias_angle_generator final
    : public csteer2D::base_bias_angle_generator {
 public:
  explicit normal_bias_angle_generator(const config::bias_angle_config* config)
      : base_bias_angle_generator(config) {}

  /* Not move/copy constructable/assignable by default */
  normal_bias_angle_generator(const normal_bias_angle_generator&) = delete;
  normal_bias_angle_generator&
  operator=(const normal_bias_angle_generator&) = delete;
  normal_bias_angle_generator(normal_bias_angle_generator&&) = delete;
  normal_bias_angle_generator& operator=(normal_bias_angle_generator&&) = delete;

  rmath::radians operator()(const rmath::radians& last_heading,
                            rmath::rng* rng) override;
};

NS_END(steer2D, cosm);
