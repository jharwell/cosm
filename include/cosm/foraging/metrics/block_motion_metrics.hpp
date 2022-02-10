/**
 * \file block_motion_metrics.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * This file is part of COSM.
 *
 * COSM is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * COSM is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * COSM.  If not, see <http://www.gnu.org/licenses/
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
NS_START(cosm, foraging, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_motion_metrics
 * \ingroup cosm foraging metrics
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

NS_END(metrics, foraging, cosm);

