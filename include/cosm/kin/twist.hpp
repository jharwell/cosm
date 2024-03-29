/**
 * \file twist.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/radians.hpp"
#include "rcppsw/math/vector3.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::kin {

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
/**
 * \struct twist
 * \ingroup kin
 *
 * \brief Representation of the twist of a robot. ROS already has this, but
 * does not work with all robotic simulators (such as ARGoS)/models, hence the
 * need.
 */
struct twist {
  rmath::vector3d linear{};
  rmath::vector3d angular{};

  /**
   * \brief Accumulate twist--should only be used in metric collection contexts.
   */
  twist& operator+=(const twist &rhs) {
    this->linear += rhs.linear;
    this->angular += rhs.angular;

    return *this;
  }

};

} /* namespace cosm::kin */
