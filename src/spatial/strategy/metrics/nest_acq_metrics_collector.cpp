/**
 * \file nest_acq_metrics_collector.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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
#include "cosm/spatial/strategy/metrics/nest_acq_metrics_collector.hpp"

#include "cosm/spatial/strategy/metrics/nest_acq_metrics.hpp"
#include "cosm/spatial/strategy/nest_acq/random_thresh.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, spatial, strategy, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
nest_acq_metrics_collector::nest_acq_metrics_collector(
    const std::string& ofname_stem,
    const rtypes::timestep& interval)
    : base_metrics_collector(ofname_stem,
                             interval,
                             rmetrics::output_mode::ekAPPEND) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string> nest_acq_metrics_collector::csv_header_cols(void) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
    /* clang-format off */
    "int_avg_random_thresh",
    "cum_avg_random_thresh",
    /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

void nest_acq_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

boost::optional<std::string> nest_acq_metrics_collector::csv_line_build(void) {
  if (!(timestep() % interval() == 0UL)) {
    return boost::none;
  }
  std::string line;
  line += csv_entry_domavg(m_interval.random_thresh.load(),
                           m_interval.n_random_thresh);
  line += csv_entry_domavg(m_cum.random_thresh.load(),
                           m_cum.n_random_thresh);

  return boost::make_optional(line);
} /* csv_line_build() */

void nest_acq_metrics_collector::collect(const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const nest_acq_metrics&>(metrics);
  auto* thresh = dynamic_cast<const cssnest_acq::random_thresh*>(m.nest_acq_strategy());

  if (nullptr == thresh) {
    return;
  }

  if (auto current = thresh->thresh()) {
    m_interval.n_random_thresh++;
    m_cum.n_random_thresh++;

    auto int_dist = m_interval.random_thresh.load();
    auto cum_dist = m_cum.random_thresh.load();
    m_interval.random_thresh.compare_exchange_strong(int_dist,
                                                     int_dist + current->v());
    m_cum.random_thresh.compare_exchange_strong(cum_dist,
                                                cum_dist + current->v());
  }
} /* collect() */

void nest_acq_metrics_collector::reset_after_interval(void) {
  m_interval.random_thresh = 0.0;
  m_interval.n_random_thresh = 0;
} /* reset_after_interval() */

NS_END(metrics, strategy, spatial, cosm);
