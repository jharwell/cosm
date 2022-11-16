/**
 * \file interference_metrics_data.hpp
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
NS_START(cosm, spatial, metrics, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
  /**
   * \brief Container for holding collected statistics. Must be atomic so counts
   * are valid in parallel metric collection contexts. Ideally the durations
   * would be atomic \ref rtypes::timestep, but that type does not meet the
   * std::atomic requirements.
   */
  struct interference_metrics_data {
    ral::mt_size_t n_exp_interference{0};
    ral::mt_size_t n_episodes{0};
    ral::mt_size_t n_entered_interference{0};
    ral::mt_size_t n_exited_interference{0};
    ral::mt_size_t interference_duration{0};
  };


NS_END(detail);

struct interference_metrics_data : public rmetrics::base_data {
  detail::interference_metrics_data interval{};
  detail::interference_metrics_data cum{};

  /**
   * \brief Accumulate data. We ignore the "cum" field on \p rhs, and accumulate
   * into our "cum" field using the "interval" field of \p rhs.
   *
   * This is the most meaningful semantics I could come up with; I couldn't find
   * a way to justify accumulating already cumulative data again (it would have
   * required some additional changes/contortions elsewhere).
   */
  interference_metrics_data& operator+=(const interference_metrics_data& rhs) {
    ral::mt_accum(this->interval.n_exp_interference,
                  rhs.interval.n_exp_interference);
    ral::mt_accum(this->interval.n_episodes,
                  rhs.interval.n_episodes);
    ral::mt_accum(this->interval.n_entered_interference,
                  rhs.interval.n_entered_interference);
    ral::mt_accum(this->interval.n_exited_interference,
                  rhs.interval.n_exited_interference);
    ral::mt_accum(this->interval.interference_duration,
                  rhs.interval.interference_duration);

    ral::mt_accum(this->cum.n_exp_interference,
                  rhs.interval.n_exp_interference);
    ral::mt_accum(this->cum.n_episodes,
                  rhs.interval.n_episodes);
    ral::mt_accum(this->cum.n_entered_interference,
                  rhs.interval.n_entered_interference);
    ral::mt_accum(this->cum.n_exited_interference,
                  rhs.interval.n_exited_interference);
    ral::mt_accum(this->cum.interference_duration,
                  rhs.interval.interference_duration);

    return *this;
  }
};

NS_END(metrics, spatial, cosm);
