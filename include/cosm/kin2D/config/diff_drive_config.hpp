/**
 * \file diff_drive_config.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
#include "rcppsw/config/base_config.hpp"
#include "rcppsw/math/radians.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, kin2D, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct diff_drive_config
 * \ingroup kin2D config
 *
 * \brief Configuration for differential drive actuator.
 */
struct diff_drive_config final : public rconfig::base_config {
  /**
   * \brief Maximum angle difference between current and new heading that will
   * not trigger a hard (in place) turn.
   */
  rmath::radians soft_turn_max{};

  /**
   * \brief The maximum angular speed. Radians/s.
   */
  double max_angular_speed{0.0};

  /**
   * \brief The maximum linear speed. M/s.
   */
  double max_linear_speed{0.0};
};

NS_END(config, kin2D, cosm);
