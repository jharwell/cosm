/**
 * \file block_transporter_metrics.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_metrics.hpp"
#include "cosm/repr/block_type.hpp"
#include "rcppsw/types/timestep.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, fsm, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_transporter_metrics
 * \ingroup fsm metrics
 *
 * \brief Defines the metrics to be collected from robots as they are
 * transporting blocks around the arena. This cannot be part of \ref
 * block_transporter, because metrics cannot be templated and also used in a
 * general way with metrics collectors.
 *
 * Metrics should be collected every timestep.
 */
class block_transporter_metrics : public virtual rmetrics::base_metrics {
 public:
  block_transporter_metrics(void) = default;

  /**
   * \brief If \c True, then the robot is moving towards its goal with its block
   * via phototaxis.
   *
   * \param include_ca If \c TRUE, then photoaxiing towards a goal is defined to
   * include collision avoidance manuvers. If \c FALSE, then phototaxiing
   * towards a goal only occurs when a robot is not also avoiding
   * collision. That is, are collision and avoidance considered separate states,
   * or not?
   */
  virtual bool is_phototaxiing_to_goal(bool include_ca) const = 0;
};

NS_END(metrics, fsm, cosm);
