/**
 * \file block_transporter_metrics.hpp
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

#ifndef INCLUDE_COSM_FSM_METRICS_BLOCK_TRANSPORTER_METRICS_HPP_
#define INCLUDE_COSM_FSM_METRICS_BLOCK_TRANSPORTER_METRICS_HPP_

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
 * \ingroup cosm fsm metrics
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
   */
  virtual bool is_phototaxiing_to_goal(void) const = 0;
};

NS_END(metrics, fsm, cosm);

#endif /* INCLUDE_COSM_FSM_METRICS_BLOCK_TRANSPORTER_METRICS_HPP_ */
