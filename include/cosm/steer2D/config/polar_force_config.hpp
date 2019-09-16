/**
 * @file polar_force_config.hpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_STEER2D_CONFIG_POLAR_FORCE_CONFIG_HPP_
#define INCLUDE_COSM_STEER2D_CONFIG_POLAR_FORCE_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/rcppsw.hpp"
#include "rcppsw/config/base_config.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, steer2D, config);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * @struct polar_force_config
 * @ingroup cosm steer2D config
 */
struct polar_force_config final : public rconfig::base_config {
  /**
   * The upper limit for polar force magnitude.
   */
  double max{0};
};

NS_END(config, steer2D, cosm);

#endif /* INCLUDE_COSM_STEER2D_CONFIG_KINEMATICS2D_CONFIG_HPP_ */
