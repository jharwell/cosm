/**
 * \file base_adaptor.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <ros/ros.h>

#include "rcppsw/mpl/reflectable.hpp"

#include "cosm/controller/base_controller.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::pal::ros::controller {

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

} /* namespace cosm::pal::ros::controller */
