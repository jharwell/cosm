/**
 * \file topic.hpp
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

#ifndef INCLUDE_COSM_ROS_TOPIC_HPP_
#define INCLUDE_COSM_ROS_TOPIC_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <filesystem>

#include "rcppsw/types/type_uuid.hpp"

#include "cosm/cosm.hpp"
#include "cosm/pal/pal.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ros);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
using topic = std::filesystem::path;

/*******************************************************************************
 * Free Functions
 ******************************************************************************/
static inline topic to_ns(const rtypes::type_uuid& robot_id) {
  return cros::topic("/" +
                     cpal::kRobotNamePrefix +
                     rcppsw::to_string(robot_id));
} /* to_ns() */

NS_END(ros, cosm);

#endif /* INCLUDE_COSM_ROS_TOPIC_HPP_ */
