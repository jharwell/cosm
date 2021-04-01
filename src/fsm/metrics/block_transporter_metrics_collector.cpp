/**
 * \file block_transporter_metrics_collector.cpp
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
    const std::string& ofname_stem,
    const rtypes::timestep& interval)
    : base_metrics_collector(ofname_stem,
                             interval,
                             rmetrics::output_mode::ekAPPEND) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string>
block_transporter_metrics_collector::csv_header_cols(void) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
    /* clang-format off */
    "int_avg_phototaxiing_to_goal_including_ca",
    "int_avg_phototaxiing_to_goal_no_ca",
    "cum_avg_phototaxiing_to_goal_including_ca",
    "cum_avg_phototaxiing_to_goal_no_ca",
    /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

void block_transporter_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

boost::optional<std::string>
block_transporter_metrics_collector::csv_line_build(void) {
  if (!(timestep() % interval() == 0UL)) {
    return boost::none;
  }
  std::string line;

  line += csv_entry_intavg(m_interval.n_phototaxiing_to_goal_including_ca);
  line += csv_entry_intavg(m_interval.n_phototaxiing_to_goal_no_ca);

  line += csv_entry_tsavg(m_cum.n_phototaxiing_to_goal_including_ca);
  line += csv_entry_tsavg(m_cum.n_phototaxiing_to_goal_no_ca);

  return boost::make_optional(line);
} /* csv_line_build() */

void block_transporter_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const block_transporter_metrics&>(metrics);
  bool including_ca = m.is_phototaxiing_to_goal(true);
  bool no_ca = m.is_phototaxiing_to_goal(false);

  m_interval.n_phototaxiing_to_goal_including_ca += including_ca;
  m_interval.n_phototaxiing_to_goal_no_ca += no_ca;

  m_cum.n_phototaxiing_to_goal_including_ca += including_ca;
  m_cum.n_phototaxiing_to_goal_no_ca += no_ca;
} /* collect() */

void block_transporter_metrics_collector::reset_after_interval(void) {
  m_interval.n_phototaxiing_to_goal_including_ca = 0;
  m_interval.n_phototaxiing_to_goal_no_ca = 0;
} /* reset_after_interval() */

NS_END(metrics, fsm, cosm);
