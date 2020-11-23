/**
 * \file interference_metrics_collector.cpp
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
#include "cosm/spatial/metrics/interference_metrics_collector.hpp"

#include "cosm/spatial/metrics/interference_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, spatial, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
interference_metrics_collector::interference_metrics_collector(
    const std::string& ofname_stem,
    const rtypes::timestep& interval)
    : base_metrics_collector(ofname_stem,
                             interval,
                             rmetrics::output_mode::ekAPPEND) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string> interference_metrics_collector::csv_header_cols(
    void) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
      /* clang-format off */
    "int_avg_exp_interference",
    "cum_avg_exp_interference",
    "int_avg_entered_interference",
    "cum_avg_entered_interference",
    "int_avg_exited_interference",
    "cum_avg_exited_interference",
    "int_avg_episodes",
    "cum_avg_episodes",
    "int_avg_interference_duration",
    "cum_avg_interference_duration"
      /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

void interference_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

void interference_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const interference_metrics&>(metrics);
  m_interval.n_exp_interference += static_cast<size_t>(m.exp_interference());
  m_cum.n_exp_interference += static_cast<size_t>(m.exp_interference());

  m_interval.n_entered_interference +=
      static_cast<size_t>(m.entered_interference());
  m_cum.n_entered_interference += static_cast<size_t>(m.entered_interference());

  m_interval.n_exited_interference += static_cast<size_t>(m.exited_interference());
  m_cum.n_exited_interference += static_cast<size_t>(m.exited_interference());

  if (m.exited_interference()) {
    ++m_interval.n_episodes;
    ++m_cum.n_episodes;

    m_interval.interference_duration += m.interference_duration().v();
    m_cum.interference_duration += m.interference_duration().v();
  }
} /* collect() */

boost::optional<std::string> interference_metrics_collector::csv_line_build(void) {
  if (!(timestep() % interval() == 0)) {
    return boost::none;
  }
  std::string line;

  line += csv_entry_intavg(m_interval.n_exp_interference.load());
  line += csv_entry_tsavg(m_cum.n_exp_interference.load());
  line += csv_entry_intavg(m_interval.n_entered_interference.load());
  line += csv_entry_tsavg(m_cum.n_entered_interference.load());
  line += csv_entry_intavg(m_interval.n_exited_interference.load());
  line += csv_entry_tsavg(m_cum.n_exited_interference.load());

  line += csv_entry_intavg(m_interval.n_episodes.load());
  line += csv_entry_tsavg(m_cum.n_episodes.load());
  line += csv_entry_domavg(m_interval.interference_duration.load(),
                           m_interval.n_episodes.load());
  line += csv_entry_domavg(m_cum.interference_duration.load(),
                           m_cum.n_episodes.load(),
                          true);
  return boost::make_optional(line);
} /* csv_line_build() */

void interference_metrics_collector::reset_after_interval(void) {
  m_interval.n_episodes = 0;
  m_interval.n_exp_interference = 0;
  m_interval.n_entered_interference = 0;
  m_interval.n_exited_interference = 0;
  m_interval.interference_duration = 0;
} /* reset_after_interval() */

NS_END(metrics, spatial, cosm);
