/**
 * \file acq_metrics.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_metrics.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::spatial::strategy::nest::acq {
class base_acq;
} /* namespace cosm::spatial::strategy::acq */

namespace cosm::spatial::strategy::nest::metrics {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class acq_metrics
 * \ingroup spatial strategy nest metrics
 *
 * \brief Metrics to be collected from \ref cssnest::acq::base_acq and derived
 * classes.
 */
class acq_metrics : virtual rmetrics::base_metrics {
 public:
  acq_metrics(void) = default;
  ~acq_metrics(void) override = default;

  /* Not move/copy constructable/assignable by default */
  acq_metrics(const acq_metrics&) = delete;
  acq_metrics& operator=(const acq_metrics&) = delete;
  acq_metrics(acq_metrics&&) = delete;
  acq_metrics& operator=(acq_metrics&&) = delete;

  /**
   * \brief Return the current nest acquisition strategy.
   */
  virtual const cssnest::acq::base_acq* nest_acq_strategy(void) const = 0;
};

} /* namespace cosm::spatial::strategy::nest::metrics */
