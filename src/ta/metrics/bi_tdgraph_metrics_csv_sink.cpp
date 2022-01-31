/**
 * \file bi_tdgraph_metrics_csv_sink.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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
#include "cosm/ta/metrics/bi_tdgraph_metrics_csv_sink.hpp"

#include "cosm/ta/metrics/bi_tdgraph_metrics_data.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
bi_tdgraph_metrics_csv_sink::bi_tdgraph_metrics_csv_sink(
        fs::path fpath_no_ext,
    const rmetrics::output_mode& mode,
    const rtypes::timestep& interval)
    : csv_sink(fpath_no_ext, mode, interval) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string> bi_tdgraph_metrics_csv_sink::csv_header_cols(
    const rmetrics::base_data* data) const {
  std::list<std::string> cols = dflt_csv_header_cols();

  auto* d = static_cast<const bi_tdgraph_metrics_data*>(data);

  for (size_t i = 0; i < d->interval.depth_counts.size(); ++i) {
    cols.push_back("int_avg_d" + rcppsw::to_string(i) + "_count");
  } /* for(i..) */

  for (size_t i = 0; i < d->cum.depth_counts.size(); ++i) {
    cols.push_back("cum_avg_d" + rcppsw::to_string(i) + "_count");
  } /* for(i..) */

  for (size_t i = 0; i < d->interval.task_counts.size(); ++i) {
    cols.push_back("int_avg_task" + rcppsw::to_string(i) + "_count");
  } /* for(i..) */

  for (size_t i = 0; i < d->cum.task_counts.size(); ++i) {
    cols.push_back("cum_avg_task" + rcppsw::to_string(i) + "_count");
  } /* for(i..) */

  for (size_t i = 0; i < d->interval.tab_counts.size(); ++i) {
    cols.push_back("int_avg_tab" + rcppsw::to_string(i) + "_count");
  } /* for(i..) */

  for (size_t i = 0; i < d->cum.tab_counts.size(); ++i) {
    cols.push_back("cum_avg_tab" + rcppsw::to_string(i) + "_count");
  } /* for(i..) */
  return cols;
} /* csv_header_cols() */

boost::optional<std::string> bi_tdgraph_metrics_csv_sink::csv_line_build(
const rmetrics::base_data* data,
      const rtypes::timestep& t) {
  if (!ready_to_flush(t)) {
    return boost::none;
  }

  auto* d = static_cast<const bi_tdgraph_metrics_data*>(data);

  std::string line;

  for (auto& count : d->interval.depth_counts) {
    line += csv_entry_intavg(count);
  } /* for(count..) */

  for (auto& count : d->cum.depth_counts) {
    line += csv_entry_tsavg(count, t);
  } /* for(&count..) */

  for (auto& count : d->interval.task_counts) {
    line += csv_entry_intavg(count);
  } /* for(&count..) */

  for (auto& count : d->cum.task_counts) {
    line += csv_entry_tsavg(count, t);
  } /* for(&count..) */

  for (auto& count : d->interval.tab_counts) {
    line += csv_entry_intavg(count);
  } /* for(&count..) */

  for (size_t i = 0; i < d->cum.tab_counts.size() - 1; ++i) {
    line += csv_entry_tsavg(d->cum.tab_counts[i], t);
  } /* for(i..) */

  line += csv_entry_tsavg(d->cum.tab_counts[d->cum.tab_counts.size() - 1],
                          t,
                          true);
  return boost::make_optional(line);
} /* csv_line_build() */


NS_END(metrics, ta, cosm);
