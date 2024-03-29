/**
 * \file battery_metrics_data.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_data.hpp"
#include "rcppsw/al/multithread.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::hal::sensors::metrics {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct battery_metrics_data_impl
 * \ingroup hal sensors metrics
 *
 * \brief Container for holding \ref battery_metrics data. Must
 * be atomic so counts are valid in parallel metric collection contexts.
 */
struct battery_metrics_data_impl {
  ral::mt_size_t n_robots{0};
  ral::mt_double_t percentage{0};
  ral::mt_double_t voltage{0};
  ral::mt_double_t time_remaining{0};
};

struct battery_metrics_data : public rmetrics::base_data {
  battery_metrics_data_impl interval{};
  battery_metrics_data_impl cum{};

  /**
   * \brief Accumulate data. We ignore the "cum" field on \p rhs, and accumulate
   * into our "cum" field using the "interval" field of \p rhs.
   *
   * This is the most meaningful semantics I could come up with; I couldn't find
   * a way to justify accumulating already cumulative data again (it would have
   * required some additional changes/contortions elsewhere).
   */
  battery_metrics_data& operator+=(const battery_metrics_data &rhs) {
    ral::mt_set(this->interval.n_robots,
                rhs.interval.n_robots);
    ral::mt_accum(this->interval.percentage,
                  rhs.interval.percentage);
    ral::mt_accum(this->interval.voltage,
                  rhs.interval.voltage);
    ral::mt_accum(this->interval.time_remaining,
                  rhs.interval.time_remaining);


    ral::mt_set(this->cum.n_robots,
                rhs.interval.n_robots);
    ral::mt_accum(this->cum.percentage,
                  rhs.interval.percentage);
    ral::mt_accum(this->cum.voltage,
                  rhs.interval.voltage);
    ral::mt_accum(this->cum.time_remaining,
                  rhs.interval.time_remaining);
    return *this;
  }
};

} /* namespace cosm::hal::sensors::metrics */
