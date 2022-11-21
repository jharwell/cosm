/**
 * \file pose.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/orientation.hpp"
#include "rcppsw/math/vector3.hpp"
#include "rcppsw/al/multithread.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::kin {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct pose
 * \ingroup kin
 *
 * \brief Representation of the pose of a robot. ROS already has this, but
 * does not work with all robotic simulators (such as ARGoS)/models, hence the
 * need.
 */
struct pose {
  rmath::vector3d position{};
  rmath::euler_angles orientation{};

  /**
   * \brief Accumulate pose--should only be used in metric collection contexts.
   */
  pose& operator+=(const pose &rhs) {
    this->position += rhs.position;
    this->orientation += rhs.orientation;

    return *this;
  }
};

} /* namespace cosm::kin */
