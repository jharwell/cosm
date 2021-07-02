/**
 * \file block_transportee_metrics_collector.cpp
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
    std::unique_ptr<rmetrics::base_metrics_sink> sink)
    : base_metrics_collector(std::move(sink)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_transportee_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  auto& m = static_cast<const block_transportee_metrics&>(metrics);

  ++m_data.interval.transported;
  m_data.interval.cube_transported +=
      static_cast<size_t>(crepr::block_type::ekCUBE == m.type());
  m_data.interval.ramp_transported +=
      static_cast<size_t>(crepr::block_type::ekRAMP == m.type());

  ++m_data.cum.transported;
  m_data.cum.cube_transported +=
      static_cast<size_t>(crepr::block_type::ekCUBE == m.type());
  m_data.cum.ramp_transported +=
      static_cast<size_t>(crepr::block_type::ekRAMP == m.type());

  m_data.interval.transporters += m.total_transporters();
  m_data.cum.transporters += m.total_transporters();

  m_data.interval.transport_time += m.total_transport_time().v();
  m_data.cum.transport_time += m.total_transport_time().v();

  m_data.interval.initial_wait_time += m.initial_wait_time().v();
  m_data.cum.initial_wait_time += m.initial_wait_time().v();
} /* collect() */

void block_transportee_metrics_collector::reset_after_interval(void) {
  m_data.interval.transported = 0;
  m_data.interval.cube_transported = 0;
  m_data.interval.ramp_transported = 0;
  m_data.interval.transporters = 0;
  m_data.interval.transport_time = 0;
  m_data.interval.initial_wait_time = 0;
} /* reset_after_interval() */

NS_END(metrics, foraging, cosm);
