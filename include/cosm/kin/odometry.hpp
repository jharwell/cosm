/**
 * \file odometry.hpp
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
#include "rcppsw/math/vector3.hpp"

#include "cosm/kin/pose.hpp"
#include "cosm/kin/twist.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, kin);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct odometry
 * \ingroup kin
 *
 * \brief Representation of the odometry of a robot. ROS already has this, but
 * does not work with all robotic simulators (such as ARGoS)/models, hence the
 * need.
 */
struct odometry {
  ckin::pose pose{};
  ckin::twist twist{};
};

NS_END(kin, cosm);
