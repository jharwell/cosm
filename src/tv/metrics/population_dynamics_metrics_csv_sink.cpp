/**
 * \file population_dynamics_metrics_csv_sink.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/tv/metrics/population_dynamics_metrics_csv_sink.hpp"

#include "cosm/tv/metrics/population_dynamics_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::tv::metrics {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
population_dynamics_metrics_csv_sink::population_dynamics_metrics_csv_sink(
    fs::path fpath_no_ext,
    const rmetrics::output_mode& mode,
    const rtypes::timestep& interval)
    : csv_sink(fpath_no_ext, mode, interval) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string> population_dynamics_metrics_csv_sink::csv_header_cols(
    const rmetrics::base_data*) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
    /* clang-format off */
    "int_avg_total_population",
    "int_avg_active_population",
    "cum_avg_total_population",
    "cum_avg_active_population",
    "max_population",

    "int_avg_birth_rate",
    "int_avg_birth_interval",
    "cum_avg_birth_rate",
    "cum_avg_birth_interval",
    "birth_mu",

    "int_avg_death_rate",
    "int_avg_death_interval",
    "cum_avg_death_rate",
    "cum_avg_death_interval",
    "death_lambda",

    "int_avg_repair_queue_size",
    "cum_avg_repair_queue_size",

    "int_avg_malfunction_rate",
    "int_avg_malfunction_interval",
    "cum_avg_malfunction_rate",
    "cum_avg_malfunction_interval",
    "malfunction_lambda",

    "int_avg_repair_rate",
    "int_avg_repair_interval",
    "cum_avg_repair_rate",
    "cum_avg_repair_interval",
    "repair_mu",
    /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

boost::optional<std::string> population_dynamics_metrics_csv_sink::csv_line_build(
    const rmetrics::base_data* data,
    const rtypes::timestep& t) {
  if (!ready_to_flush(t)) {
    return boost::none;
  }
  auto* d = static_cast<const population_dynamics_metrics_data*>(data);
  std::string line;

  /* population */
  line += csv_entry_intavg(d->interval.total_population);
  line += csv_entry_intavg(d->interval.active_population);
  line += csv_entry_tsavg(d->cum.total_population, t);
  line += csv_entry_tsavg(d->cum.active_population, t);
  line += rcppsw::to_string(d->interval.max_population) + separator();

  /* birth queue */
  line += csv_entry_intavg(d->interval.n_births);
  line += csv_entry_domavg(d->interval.birth_interval, d->interval.n_births);

  line += csv_entry_tsavg(d->cum.n_births, t);
  line += csv_entry_domavg(d->cum.birth_interval, d->cum.n_births);

  line += rcppsw::to_string(d->interval.birth_mu) + separator();

  /* death queue */
  line += csv_entry_intavg(ral::mt_load(d->interval.n_deaths));
  line += csv_entry_domavg(ral::mt_load(d->interval.death_interval),
                           ral::mt_load(d->interval.n_deaths));

  line += csv_entry_tsavg(ral::mt_load(d->cum.n_deaths), t);
  line += csv_entry_domavg(ral::mt_load(d->cum.death_interval),
                           ral::mt_load(d->cum.n_deaths));
  line += rcppsw::to_string(ral::mt_load(d->interval.death_lambda)) + separator();

  /* repair queue */
  line += csv_entry_intavg(d->interval.repair_queue_size);
  line += csv_entry_tsavg(d->cum.repair_queue_size, t);

  /* repair queue malfunctions */
  line += csv_entry_intavg(d->interval.n_malfunctions);
  line += csv_entry_domavg(d->interval.malfunction_interval,
                           d->interval.n_malfunctions);

  line += csv_entry_tsavg(d->cum.n_malfunctions, t);
  line += csv_entry_domavg(d->cum.malfunction_interval, d->cum.n_malfunctions);
  line += rcppsw::to_string(d->interval.malfunction_lambda) + separator();

  /* repair queue repairs */
  line += csv_entry_intavg(d->interval.n_repairs);
  line += csv_entry_domavg(d->interval.repair_interval, d->interval.n_repairs);

  line += csv_entry_tsavg(d->cum.n_repairs, t);
  line += csv_entry_domavg(d->cum.repair_interval, d->cum.n_repairs);
  line += rcppsw::to_string(d->interval.repair_mu);

  return boost::make_optional(line);
} /* csv_line_build() */

} /* namespace cosm::tv::metrics */
