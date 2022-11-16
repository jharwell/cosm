/**
 * \file bias_angle_generator_factory.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/patterns/factory/factory.hpp"

#include "cosm/cosm.hpp"
#include "cosm/steer2D/base_bias_angle_generator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, steer2D);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class bias_angle_generator_factory
 * \ingroup steer2D
 *
 * \brief Factory for creating bias angle generators for \ref wander_force.
 */
class bias_angle_generator_factory
    : public rpfactory::releasing_factory<
          base_bias_angle_generator,
          std::string, /* key type */
          const csteer2D::config::bias_angle_config*> {
 public:
  /**
   * \brief Angles will be drawn from a uniform distribution between [-max bias,
   * max bias].
   */
  static inline const std::string kUniform = "uniform";

  /**
   * \brief Angles will be drawn from a normal distribution N(0, max_bias / 3).
   */
  static inline const std::string kNormal = "normal";

  /**
   * \brief Angles will be drawn uniformly from the specified custom
   * distribution.
   */
  static inline const std::string kCustom = "custom";

  bias_angle_generator_factory(void);
};

NS_END(steer2D, cosm);
