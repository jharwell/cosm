/**
 * \file block_motion_metrics_collector.cpp
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
#include "cosm/foraging/metrics/block_motion_metrics_collector.hpp"

#include "cosm/foraging/metrics/block_motion_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_motion_metrics_collector::block_motion_metrics_collector(
    const std::string& ofname_stem,
    const rtypes::timestep& interval)
    : base_metrics_collector(ofname_stem,
                             interval,
                             rmetrics::output_mode::ekAPPEND) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string>
block_motion_metrics_collector::csv_header_cols(void) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
    /* clang-format off */
    "int_avg_blocks_moved",
    "cum_avg_blocks_moved",
    /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

void block_motion_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

boost::optional<std::string>
block_motion_metrics_collector::csv_line_build(void) {
  if (!(timestep() % interval() == 0)) {
    return boost::none;
  }
  std::string line;

  line += csv_entry_intavg(m_interval.n_moved);
  line += csv_entry_tsavg(m_cum.n_moved, true);

  return boost::make_optional(line);
} /* csv_line_build() */

void block_motion_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  auto& m = static_cast<const block_motion_metrics&>(metrics);
  m_interval.n_moved += m.n_moved();
  m_cum.n_moved += m.n_moved();
} /* collect() */

void block_motion_metrics_collector::reset_after_interval(void) {
  m_interval.n_moved = 0;
} /* reset_after_interval() */

NS_END(metrics, foraging, cosm);
