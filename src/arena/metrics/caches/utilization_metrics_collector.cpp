/**
 * \file utilization_metrics_collector.cpp
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
#include "cosm/arena/metrics/caches/utilization_metrics_collector.hpp"

#include "cosm/arena/metrics/caches/utilization_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, arena, metrics, caches);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
utilization_metrics_collector::utilization_metrics_collector(
    const std::string& ofname_stem,
    const rtypes::timestep& interval)
    : base_metrics_collector(ofname_stem,
                             interval,
                             rmetrics::output_mode::ekAPPEND) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string>
utilization_metrics_collector::csv_header_cols(void) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
    /* clang-format off */
    "int_avg_blocks",
    "cum_avg_blocks",
    "int_avg_pickups",
    "cum_avg_pickups",
    "int_avg_drops" ,
    "cum_avg_drops" ,
    "int_avg_caches",
    "cum_avg_caches"
    /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

void utilization_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

boost::optional<std::string> utilization_metrics_collector::csv_line_build(void) {
  if (!(timestep() % interval() == 0)) {
    return boost::none;
  }
  std::string line;

  line += csv_entry_domavg(m_interval.n_blocks, m_interval.cache_count);
  line += csv_entry_domavg(m_cum.n_blocks, m_cum.cache_count);
  line += csv_entry_domavg(m_interval.n_pickups, m_interval.cache_count);
  line += csv_entry_domavg(m_cum.n_pickups, m_cum.cache_count);
  line += csv_entry_domavg(m_interval.n_drops, m_interval.cache_count);
  line += csv_entry_domavg(m_cum.n_drops, m_cum.cache_count);
  line += csv_entry_intavg(m_interval.cache_count);
  line += csv_entry_tsavg(m_cum.cache_count, true);

  return boost::make_optional(line);
} /* csv_line_build() */

void utilization_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const utilization_metrics&>(metrics);

  m_interval.n_pickups += m.total_block_pickups();
  m_interval.n_drops += m.total_block_drops();
  m_interval.n_blocks += m.n_blocks();

  m_cum.n_pickups += m.total_block_pickups();
  m_cum.n_drops += m.total_block_drops();
  m_cum.n_blocks += m.n_blocks();

  ++m_interval.cache_count;
  ++m_cum.cache_count;
} /* collect() */

void utilization_metrics_collector::reset_after_interval(void) {
  m_interval.n_blocks = 0;
  m_interval.n_pickups = 0;
  m_interval.n_drops = 0;
  m_interval.cache_count = 0;
} /* reset_after_interval() */

NS_END(caches, metrics, arena, cosm);
