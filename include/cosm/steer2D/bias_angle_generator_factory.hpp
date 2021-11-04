/**
 * \file bias_angle_generator_factory.hpp
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

#ifndef INCLUDE_COSM_STEER2D_ANGLE_GENERATOR_FACTORY_HPP_
#define INCLUDE_COSM_STEER2D_ANGLE_GENERATOR_FACTORY_HPP_

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
class bias_angle_generator_factory :
    public rpfactory::releasing_factory<base_bias_angle_generator,
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

#endif /* INCLUDE_COSM_STEER2D_ANGLE_GENERATOR_FACTORY_HPP_ */
