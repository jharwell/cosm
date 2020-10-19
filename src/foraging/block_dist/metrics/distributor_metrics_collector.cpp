/**
 * \file distributor_metrics_collector.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/foraging/block_dist/metrics/distributor_metrics_collector.hpp"

#include "cosm/foraging/block_dist/metrics/distributor_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, block_dist, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
distributor_metrics_collector::distributor_metrics_collector(
    const std::string& ofname_stem,
    const rtypes::timestep& interval)
    : base_metrics_collector(ofname_stem,
                             interval,
                             rmetrics::output_mode::ekAPPEND) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string> distributor_metrics_collector::csv_header_cols(void) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
      /* clang-format off */
    "n_configured_clusters",
    "n_mapped_clusters",
    "capacity",
    "int_avg_size",
    "cum_avg_size",
      /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

void distributor_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

boost::optional<std::string> distributor_metrics_collector::csv_line_build(void) {
  if (!(timestep() % interval() == 0)) {
    return boost::none;
  }
  std::string line;

  line += rcppsw::to_string(m_cum.n_configured_clusters) + separator();
  line += rcppsw::to_string(m_cum.n_mapped_clusters) + separator();
  line += rcppsw::to_string(m_cum.capacity) + separator();

  line += csv_entry_intavg(m_interval.size);
  line += csv_entry_tsavg(m_cum.size);

  return boost::make_optional(line);
} /* csv_line_build() */

void distributor_metrics_collector::collect(const rmetrics::base_metrics& metrics) {
  auto& m = static_cast<const distributor_metrics&>(metrics);

  m_interval.n_configured_clusters = m.n_configured_clusters();
  m_interval.n_mapped_clusters = m.n_mapped_clusters();
  m_interval.capacity = m.capacity();
  m_interval.size += m.size();

  m_cum.n_configured_clusters = m.n_configured_clusters();
  m_cum.n_mapped_clusters = m.n_mapped_clusters();
  m_cum.capacity = m.capacity();
  m_cum.size += m.size();
} /* collect() */

void distributor_metrics_collector::reset_after_interval(void) {
  m_interval.size = 0;
} /* reset_after_interval() */

NS_END(metrics, block_dist, foraging, cosm);
