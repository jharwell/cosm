/**
 * \file transport_metrics_collector.cpp
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
#include "cosm/metrics/blocks/transport_metrics_collector.hpp"

#include "cosm/metrics/blocks/transport_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, metrics, blocks);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
transport_metrics_collector::transport_metrics_collector(
    const std::string& ofname_stem,
    const rtypes::timestep& interval)
    : base_metrics_collector(ofname_stem,
                             interval,
                             rmetrics::output_mode::ekAPPEND) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string> transport_metrics_collector::csv_header_cols(void) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
      /* clang-format off */
    "cum_transported",
    "cum_ramp_transported",
    "cum_cube_transported",
    "int_avg_transported",
    "cum_avg_transported",
    "int_avg_cube_transported",
    "cum_avg_cube_transported",
    "int_avg_ramp_transported",
    "cum_avg_ramp_transported",
    "int_avg_transporters",
    "cum_avg_transporters",
    "int_avg_transport_time",
    "cum_avg_transport_time",
    "int_avg_initial_wait_time",
    "cum_avg_initial_wait_time"
      /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

void transport_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

boost::optional<std::string> transport_metrics_collector::csv_line_build(void) {
  if (!(timestep() % interval() == 0)) {
    return boost::none;
  }
  std::string line;

  line += rcppsw::to_string(m_cum.transported) + separator();
  line += rcppsw::to_string(m_cum.ramp_transported) + separator();
  line += rcppsw::to_string(m_cum.cube_transported) + separator();

  line += csv_entry_intavg(m_interval.transported);
  line += csv_entry_tsavg(m_cum.transported);

  line += csv_entry_intavg(m_interval.cube_transported);
  line += csv_entry_tsavg(m_cum.cube_transported);
  line += csv_entry_intavg(m_interval.ramp_transported);
  line += csv_entry_tsavg(m_cum.ramp_transported);
  line += csv_entry_domavg(m_interval.transporters, m_interval.transported);
  line += csv_entry_domavg(m_cum.transporters, m_cum.transported);

  line += csv_entry_domavg(m_interval.transport_time, m_interval.transported);
  line += csv_entry_domavg(m_cum.transport_time, m_cum.transported);

  line += csv_entry_domavg(m_interval.initial_wait_time, m_interval.transported);
  line += csv_entry_domavg(m_cum.initial_wait_time, m_cum.transported, true);

  return boost::make_optional(line);
} /* csv_line_build() */

void transport_metrics_collector::collect(const rmetrics::base_metrics& metrics) {
  auto& m = static_cast<const transport_metrics&>(metrics);
  ++m_interval.transported;
  m_interval.cube_transported += static_cast<uint>(repr::block_type::ekCUBE == m.type());
  m_interval.ramp_transported += static_cast<uint>(repr::block_type::ekRAMP == m.type());

  ++m_cum.transported;
  m_cum.cube_transported += static_cast<uint>(repr::block_type::ekCUBE == m.type());
  m_cum.ramp_transported += static_cast<uint>(repr::block_type::ekRAMP == m.type());

  m_interval.transporters += m.total_transporters();
  m_cum.transporters += m.total_transporters();

  m_interval.transport_time += m.total_transport_time().v();
  m_cum.transport_time += m.total_transport_time().v();

  m_interval.initial_wait_time += m.initial_wait_time().v();
  m_cum.initial_wait_time += m.initial_wait_time().v();
} /* collect() */

void transport_metrics_collector::reset_after_interval(void) {
  m_interval.transported = 0;
  m_interval.cube_transported = 0;
  m_interval.ramp_transported = 0;
  m_interval.transporters = 0;
  m_interval.transport_time = 0;
  m_interval.initial_wait_time = 0;
} /* reset_after_interval() */

NS_END(blocks, metrics, cosm);
