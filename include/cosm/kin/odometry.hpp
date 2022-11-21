/**
 * \file odometry.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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
namespace cosm::kin {

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

} /* namespace cosm::kin */
