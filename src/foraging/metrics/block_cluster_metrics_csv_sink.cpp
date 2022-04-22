/**
 * \file block_cluster_metrics_collector.cpp
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
#include "cosm/foraging/metrics/block_cluster_metrics_csv_sink.hpp"

#include "cosm/foraging/metrics/block_cluster_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_cluster_metrics_csv_sink::block_cluster_metrics_csv_sink(
    fs::path fpath_no_ext,
    const rmetrics::output_mode& mode,
    const rtypes::timestep& interval)
    : csv_sink(fpath_no_ext, mode, interval) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string> block_cluster_metrics_csv_sink::csv_header_cols(
    const rmetrics::base_data* data) const {
  auto cols = dflt_csv_header_cols();

  auto* d = static_cast<const block_cluster_metrics_data*>(data);

  for (size_t i = 0; i < d->interval.block_counts.size(); ++i) {
    cols.push_back("int_avg_cluster" + rcppsw::to_string(i) + "_block_count");
  } /* for(i..) */

  for (size_t i = 0; i < d->cum.block_counts.size(); ++i) {
    cols.push_back("cum_avg_cluster" + rcppsw::to_string(i) + "_block_count");
  } /* for(i..) */

  for (size_t i = 0; i < d->extents.size(); ++i) {
    cols.push_back("cluster" + rcppsw::to_string(i) + "_area");
    cols.push_back("cluster" + rcppsw::to_string(i) + "_xmin");
    cols.push_back("cluster" + rcppsw::to_string(i) + "_xmax");
    cols.push_back("cluster" + rcppsw::to_string(i) + "_ymin");
    cols.push_back("cluster" + rcppsw::to_string(i) + "_ymax");
  } /* for(i..) */

  return cols;
} /* csv_header_cols() */

boost::optional<std::string>
block_cluster_metrics_csv_sink::csv_line_build(const rmetrics::base_data* data,
                                               const rtypes::timestep& t) {
  if (!ready_to_flush(t)) {
    return boost::none;
  }
  std::string line;

  auto* d = static_cast<const block_cluster_metrics_data*>(data);

  for (auto& count : d->interval.block_counts) {
    line += csv_entry_intavg(count);
  } /* for(&count..) */

  for (auto& count : d->cum.block_counts) {
    line += csv_entry_tsavg(count, t);
  } /* for(&count..) */

  for (size_t i = 0; i < d->extents.size() - 1; ++i) {
    line += rcppsw::to_string(d->extents[i].area) + separator();
    line += rcppsw::to_string(d->extents[i].xmin) + separator();
    line += rcppsw::to_string(d->extents[i].xmax) + separator();
    line += rcppsw::to_string(d->extents[i].ymin) + separator();
    line += rcppsw::to_string(d->extents[i].ymax) + separator();
  } /* for(i..) */

  line += rcppsw::to_string(d->extents[d->extents.size() - 1].area) + separator();
  line += rcppsw::to_string(d->extents[d->extents.size() - 1].xmin) + separator();
  line += rcppsw::to_string(d->extents[d->extents.size() - 1].xmax) + separator();
  line += rcppsw::to_string(d->extents[d->extents.size() - 1].ymin) + separator();
  line += rcppsw::to_string(d->extents[d->extents.size() - 1].ymax);

  return boost::make_optional(line);
} /* csv_line_build() */

NS_END(metrics, foraging, cosm);
