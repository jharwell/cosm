/**
 * \file robot_dynamics_applicator_config.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_TV_CONFIG_ROBOT_DYNAMICS_APPLICATOR_CONFIG_HPP_
#define INCLUDE_COSM_TV_CONFIG_ROBOT_DYNAMICS_APPLICATOR_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"
#include "rcppsw/control/config/waveform_config.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, tv, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct robot_dynamics_applicator_config
 * \ingroup tv config
 *
 * \brief Configuration for the (\ref robot_dynamics_applicator).
 */
struct robot_dynamics_applicator_config final : public rconfig::base_config {
  rct::config::waveform_config motion_throttle{};
};

NS_END(config, tv, cosm);

#endif /* INCLUDE_COSM_TV_CONFIG_ROBOT_DYNAMICS_APPLICATOR_CONFIG_HPP_ */
