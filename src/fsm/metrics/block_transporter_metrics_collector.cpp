/**
 * \file block_transporter_metrics_collector.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/fsm/metrics/block_transporter_metrics_collector.hpp"

#include "cosm/fsm/metrics/block_transporter_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, fsm, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_transporter_metrics_collector::block_transporter_metrics_collector(
    std::unique_ptr<rmetrics::base_sink> sink)
    : base_collector(std::move(sink)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_transporter_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const block_transporter_metrics&>(metrics);
  bool including_ca = m.is_phototaxiing_to_goal(true);
  bool no_ca = m.is_phototaxiing_to_goal(false);

  m_data.interval.n_phototaxiing_to_goal_including_ca += including_ca;
  m_data.interval.n_phototaxiing_to_goal_no_ca += no_ca;

  m_data.cum.n_phototaxiing_to_goal_including_ca += including_ca;
  m_data.cum.n_phototaxiing_to_goal_no_ca += no_ca;
} /* collect() */

void block_transporter_metrics_collector::reset_after_interval(void) {
  m_data.interval.n_phototaxiing_to_goal_including_ca = 0;
  m_data.interval.n_phototaxiing_to_goal_no_ca = 0;
} /* reset_after_interval() */

NS_END(metrics, fsm, cosm);
