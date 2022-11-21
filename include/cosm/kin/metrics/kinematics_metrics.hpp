/**
 * \file kinematics_metrics.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/optional.hpp>

#include "rcppsw/metrics/base_metrics.hpp"
#include "rcppsw/math/vector3.hpp"
#include "rcppsw/spatial/euclidean_dist.hpp"
#include "rcppsw/types/type_uuid.hpp"
#include "rcppsw/metrics/context.hpp"

#include "cosm/cosm.hpp"
#include "cosm/kin/twist.hpp"
#include "cosm/kin/pose.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::kin::metrics {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class kinematics_metrics
 * \ingroup kin metrics
 *
 * \brief Interface defining what metrics regarding kinematics/distance traveled
 * should be collected from all controllers.
 */
class kinematics_metrics : virtual public rmetrics::base_metrics {
 public:
  kinematics_metrics(void) = default;
  ~kinematics_metrics(void) override = default;

  /**
   * \brief Get the distance that a robot has traveled in a single timestep. If
   * the robot's motion does not fall into the specified category this timestep,
   * then return nothing.
   *
   * \param ctx The category of kinematics to get the distance traveled for.
   */
  virtual boost::optional<rspatial::euclidean_dist> traveled(
      const rmetrics::context& ctx) const = 0;

  /**
   * \brief Get the twist that a robot has on a single timestep. If the robot
   * is only moving in 2D, then the Z component of the twist should be 0. If
   * the robot's motion does not fall into the specified category this timestep,
   * then the return twist should be 0 for all fields.
   *
   * \param ctx The category of motion to get the currently velocity for.
   */
  virtual boost::optional<ckin::twist> twist(
      const rmetrics::context& ctx) const = 0;

  /**
   * \brief Get the robot's current pose.
   */
  virtual ckin::pose pose(void) const = 0;


  /**
   * \brief Get the UUID for the robot.
   */
  virtual const rtypes::type_uuid& id(void) const = 0;
};

} /* namespace cosm::kin::metrics */
