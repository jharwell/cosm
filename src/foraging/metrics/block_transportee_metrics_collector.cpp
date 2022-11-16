/**
 * \file block_transportee_metrics_collector.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/foraging/metrics/block_transportee_metrics_collector.hpp"

#include "cosm/foraging/metrics/block_transportee_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_transportee_metrics_collector::block_transportee_metrics_collector(
    std::unique_ptr<rmetrics::base_sink> sink)
    : base_collector(std::move(sink)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_transportee_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  auto& m = static_cast<const block_transportee_metrics&>(metrics);

  ++m_data.interval.n_transported;
  m_data.interval.n_cube_transported +=
      static_cast<size_t>(crepr::block_type::ekCUBE == m.type());
  m_data.interval.n_ramp_transported +=
      static_cast<size_t>(crepr::block_type::ekRAMP == m.type());

  ++m_data.cum.n_transported;
  m_data.cum.n_cube_transported +=
      static_cast<size_t>(crepr::block_type::ekCUBE == m.type());
  m_data.cum.n_ramp_transported +=
      static_cast<size_t>(crepr::block_type::ekRAMP == m.type());

  m_data.interval.n_transporters += m.total_transporters();
  m_data.cum.n_transporters += m.total_transporters();

  m_data.interval.transport_time += m.total_transport_time().v();
  m_data.cum.transport_time += m.total_transport_time().v();

  m_data.interval.initial_wait_time += m.initial_wait_time().v();
  m_data.cum.initial_wait_time += m.initial_wait_time().v();
} /* collect() */

void block_transportee_metrics_collector::reset_after_interval(void) {
  m_data.interval.n_transported = 0;
  m_data.interval.n_cube_transported = 0;
  m_data.interval.n_ramp_transported = 0;
  m_data.interval.n_transporters = 0;
  m_data.interval.transport_time = 0;
  m_data.interval.initial_wait_time = 0;
} /* reset_after_interval() */

NS_END(metrics, foraging, cosm);
