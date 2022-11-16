/**
 * \file manipulation_metrics.hpp
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
#include "cosm/cosm.hpp"
#include "rcppsw/types/timestep.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, controller, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class manipulation_metrics
 * \ingroup controller metrics
 *
 * \brief Defines the metrics to be collected from controllers as they
 * manipulate their environment ( block pickup, block drop, etc.)
 */
class manipulation_metrics : public virtual rmetrics::base_metrics {
 public:
  manipulation_metrics(void) = default;
  ~manipulation_metrics(void) override = default;

  /**
   * \brief If \c TRUE, then the specified event occurred this timestep.
   */
  virtual bool status(uint event) const = 0;

  /**
   * \brief The penalty the robot was subjected to for which calling \ref
   * status() with the specified event returned \c true.
   */
  virtual rtypes::timestep penalty(uint event) const = 0;
};

NS_END(metrics, controller, cosm);
