/**
 * @file twist.hpp
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

#ifndef INCLUDE_COSM_KIN_TWIST_HPP_
#define INCLUDE_COSM_KIN_TWIST_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/rcppsw.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, kin);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/

/**
 * @struct twist
 * @ingroup cosm kin
 *
 * @brief Representation of the twist of a  robot. ROS already has this, but
 * does not work with all robotic simulators (such as ARGoS)/models, hence the
 * need for me to implement this.
 */
struct twist {
  struct {
    double x{};
    double y{};
    double z{};
  } linear{};
  struct {
    double x{};
    double y{};
    double z{};
  } angular{};
};

NS_END(kin, cosm);

#endif /* INCLUDE_COSM_KIN_TWIST_HPP_ */
