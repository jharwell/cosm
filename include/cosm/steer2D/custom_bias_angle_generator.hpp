/**
 * \file custom_bias_angle_generator.hpp
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

#ifndef INCLUDE_COSM_STEER2D_CUSTOM_BIAS_ANGLE_GENERATOR_HPP_
#define INCLUDE_COSM_STEER2D_CUSTOM_BIAS_ANGLE_GENERATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/steer2D/config/bias_angle_config.hpp"
#include "cosm/steer2D/base_bias_angle_generator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, steer2D);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class custom_bias_angle_generator
 * \ingroup steer2D
 *
 * \brief Generates bias angles for the \ref wander_force drawn from a custom
 * distribution.
 */
class custom_bias_angle_generator : public csteer2D::base_bias_angle_generator {
 public:
  explicit custom_bias_angle_generator(const config::bias_angle_config* config):
      base_bias_angle_generator(config) {}

  /* Not move/copy constructable/assignable by default */
  custom_bias_angle_generator(const custom_bias_angle_generator&) = delete;
  custom_bias_angle_generator& operator=(const custom_bias_angle_generator&) = delete;
  custom_bias_angle_generator(custom_bias_angle_generator&&) = delete;
  custom_bias_angle_generator& operator=(custom_bias_angle_generator&&) = delete;

  rmath::radians operator()(const rmath::radians&,
                            rmath::rng* rng) override;
};

NS_END(steer2D, cosm);

#endif /* INCLUDE_COSM_STEER2D_CUSTOM_BIAS_ANGLE_GENERATOR_HPP_ */