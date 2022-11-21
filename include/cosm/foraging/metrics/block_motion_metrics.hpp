/**
 * \file block_motion_metrics.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
 * Namespaces
 ******************************************************************************/
namespace cosm::foraging::metrics {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_motion_metrics
 * \ingroup foraging metrics
 *
 * \brief Defines the metrics to be collected from \ref block_motion_handler
 * about block as they move in the arena after distribution.
 *
 * Metrics should be collected every timestep.
 */
class block_motion_metrics : public rmetrics::base_metrics {
 public:
  block_motion_metrics(void) = default;

  /**
   * \brief Return the total # of blocks which have been moved this timestep.
   */
  virtual size_t n_moved(void) const = 0;
};

} /* namespace cosm::foraging::metrics */

