/**
 * \file battery_metrics.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
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
namespace cosm::hal::sensors::metrics {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class battery_metrics
 * \ingroup hal sensors metrics
 *
 * \brief Data to gather from robot battery sensors.
 */
class battery_metrics : public rmetrics::base_metrics {
 public:
  battery_metrics(void) = default;
  ~battery_metrics(void) = default;

  /**
   * \brief Get the % battery remaining as a number [0.1].
   */
  virtual double percent_remaining(void) const = 0;
};

} /* namespace cosm::hal::sensors::metrics */
