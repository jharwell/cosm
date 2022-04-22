/**
 * \file bias_angle_generator_factory.cpp
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
