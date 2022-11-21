/**
 * \file interference_metrics.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/optional.hpp>

#include "rcppsw/metrics/base_metrics.hpp"
#include "rcppsw/types/timestep.hpp"
#include "rcppsw/math/vector3.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::spatial::metrics {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class interference_metrics
 * \ingroup spatial metrics
 *
 * \brief Interface defining what metrics should be collected regarding
 * inter-robot interference as robots go about their tasks in the arena.
 */
class interference_metrics : public virtual rmetrics::base_metrics {
 public:
  interference_metrics(void) = default;
  ~interference_metrics(void) override = default;

  /**
   * \brief If \c TRUE, then a robot is currently experiencing inter-robot
   * interference.
   */
  virtual bool exp_interference(void) const = 0;

  /**
   * \brief If \c TRUE, then a robot has just begun experiencing inter-robot
   * interference. This should return \c FALSE on all subsequent steps the robot
   * is experiencing interference.
   */
  virtual bool entered_interference(void) const = 0;

  /**
   * \brief If \c TRUE, then a robot has just exited interference avoidance. This
   * should return \c FALSE on all previous steps the robot is in interference
   * avoidance, and all steps afterwards when it returns to normal operation.
   */
  virtual bool exited_interference(void) const = 0;

  /**
   * \brief If \ref exited_interference() returns \c TRUE, then this should
   * return the duration of the interference in timesteps.
   */
  virtual boost::optional<rtypes::timestep> interference_duration(void) const = 0;

  /**
   * \brief When \ref exp_interference() returns \c TRUE, then this should
   * return the robot's current discrete position in 3D.
   */
  virtual boost::optional<rmath::vector3z> interference_loc3D(void) const = 0;

};

} /* namespace cosm::spatial::metrics */
