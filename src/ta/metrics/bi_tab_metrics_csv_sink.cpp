/**
 * \file bi_tab_metrics_csv_sink.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ta/metrics/bi_tab_metrics_csv_sink.hpp"

#include "cosm/ta/metrics/bi_tab_metrics_data.hpp"
#include "cosm/ta/polled_task.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ta::metrics {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
bi_tab_metrics_csv_sink::bi_tab_metrics_csv_sink(fs::path fpath_no_ext,
                                                 const rmetrics::output_mode& mode,
                                                 const rtypes::timestep& interval)
    : csv_sink(fpath_no_ext, mode, interval) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string>
bi_tab_metrics_csv_sink::csv_header_cols(const rmetrics::base_data*) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
    /* clang-format off */
      "int_avg_subtask1_count",
      "cum_avg_subtask1_count",
      "int_avg_subtask2_count",
      "cum_avg_subtask2_count",
      "int_avg_partition_count",
      "cum_avg_partition_count",
      "int_avg_no_partition_count",
      "cum_avg_no_partition_count",
      "int_avg_task_sw_count",
      "cum_avg_task_sw_count",
      "int_avg_task_depth_sw_count",
      "cum_avg_task_depth_sw_count",
      "int_avg_partition_prob",
      "cum_avg_partition_prob",
      "int_avg_subtask_selection_prob",
      "cum_avg_subtask_selection_prob"
    /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

boost::optional<std::string>
bi_tab_metrics_csv_sink::csv_line_build(const rmetrics::base_data* data,
                                        const rtypes::timestep& t) {
  if (!ready_to_flush(t)) {
    return boost::none;
  }

  auto* d = static_cast<const bi_tab_metrics_data*>(data);

  /*
   * We want to capture average probability per robot, not per
   * timestep/interval, so we divide by the total # of task allocations
   * performed (# partitions + # no partitions).
   */
  double int_allocs =
      d->interval.partition_count + d->interval.no_partition_count;
  double cum_allocs = d->cum.partition_count + d->cum.no_partition_count;
  std::string line;

  line += csv_entry_domavg(d->interval.subtask1_count, int_allocs);
  line += csv_entry_domavg(d->cum.subtask1_count, cum_allocs);

  line += csv_entry_domavg(d->interval.subtask2_count, int_allocs);
  line += csv_entry_domavg(d->cum.subtask2_count, cum_allocs);

  line += csv_entry_domavg(d->interval.partition_count, int_allocs);
  line += csv_entry_domavg(d->cum.partition_count, cum_allocs);

  line += csv_entry_domavg(d->interval.no_partition_count, int_allocs);
  line += csv_entry_domavg(d->cum.no_partition_count, cum_allocs);

  line += csv_entry_domavg(d->interval.task_sw_count, int_allocs);
  line += csv_entry_domavg(d->cum.task_sw_count, cum_allocs);

  line += csv_entry_domavg(d->interval.task_depth_sw_count, int_allocs);
  line += csv_entry_domavg(d->cum.task_depth_sw_count, cum_allocs);

  line += csv_entry_domavg(d->interval.partition_prob, int_allocs);
  line += csv_entry_domavg(d->cum.partition_prob, cum_allocs);

  line += csv_entry_domavg(d->interval.subtask_sel_prob, int_allocs);
  line += csv_entry_domavg(d->cum.subtask_sel_prob, cum_allocs, true);

  return boost::make_optional(line);
} /* csv_line_build() */

} /* namespace cosm::ta::metrics */
