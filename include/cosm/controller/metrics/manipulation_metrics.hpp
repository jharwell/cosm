/**
 * \file manipulation_metrics.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_CONTROLLER_METRICS_MANIPULATION_METRICS_HPP_
#define INCLUDE_COSM_CONTROLLER_METRICS_MANIPULATION_METRICS_HPP_

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

#endif /* INCLUDE_COSM_CONTROLLER_METRICS_MANIPULATION_METRICS_HPP_ */
