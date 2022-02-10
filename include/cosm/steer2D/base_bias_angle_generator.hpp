/**
 * \file base_bias_angle_generator.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/math/radians.hpp"
#include "rcppsw/math/rng.hpp"

#include "cosm/steer2D/config/bias_angle_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, steer2D);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_bias_angle_generator
 * \ingroup steer2D
 *
 * \brief Generates bias angles for the \ref wander_force based on
 * configuration.
 */
class base_bias_angle_generator : public rer::client<base_bias_angle_generator> {
 public:
  explicit base_bias_angle_generator(
      const config::bias_angle_config* config)
      :  ER_CLIENT_INIT("cosm.steer2D.bias_angle_generator"),
         mc_config(*config) {}

  /* Not move/copy constructable/assignable by default */
  base_bias_angle_generator(const base_bias_angle_generator&) = delete;
  base_bias_angle_generator& operator=(const base_bias_angle_generator&) = delete;
  base_bias_angle_generator(base_bias_angle_generator&&) = delete;
  base_bias_angle_generator& operator=(base_bias_angle_generator&&) = delete;

  /**
   * \brief Generate a bias angle according to configuration.
   *
   * \param last_heading The heading angle the last time a bias angle was
   *                     generated.
   */
  virtual rmath::radians operator()(const rmath::radians& last_heading,
                                    rmath::rng* rng) = 0;

 protected:
  const config::bias_angle_config* config(void) const { return &mc_config; }

 private:
  /* clang-format off */
  const config::bias_angle_config mc_config;
  /* clang-format on */
};

NS_END(steer2D, cosm);

