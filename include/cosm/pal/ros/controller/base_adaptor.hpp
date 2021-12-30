/**
 * \file base_adaptor.hpp
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

#ifndef INCLUDE_COSM_PAL_ROS_CONTROLLER_BASE_ADAPTOR_HPP_
#define INCLUDE_COSM_PAL_ROS_CONTROLLER_BASE_ADAPTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <ros/ros.h>

#include "rcppsw/mpl/reflectable.hpp"

#include "cosm/controller/base_controller.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, pal, ros, controller);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_adaptor
 * \ingroup pal ros controller
 *
 * \brief Stub adaptor class to provide an interface for creating controllers
 * within ROS.
 */
class base_adaptor : public rmpl::reflectable {
 protected:
  const ::ros::NodeHandle* node_handle(void) const { return &m_nh; }
  ::ros::NodeHandle* node_handle(void) { return &m_nh; }

 private:
  /* clang-format off */
  ::ros::NodeHandle m_nh{};
  /* clang-format on */
};

NS_END(controller, ros, pal, cosm);

#endif /* INCLUDE_COSM_PAL_ROS_CONTROLLER_BASE_ADAPTOR_HPP_ */
