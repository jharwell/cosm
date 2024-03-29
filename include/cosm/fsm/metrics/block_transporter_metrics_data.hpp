/**
 * \file block_transporter_metrics_data.hpp
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
namespace cosm::fsm::metrics {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct block_transporter_metrics_data_impl
 * \ingroup fsm metrics detail
 *
 * \brief Container for holding \ref block_transporter_metrics data. Must
 * be atomic so counts are valid in parallel metric collection contexts.
 */
struct block_transporter_metrics_data_impl {
  ral::mt_size_t n_phototaxiing_to_goal_including_ca{0};
  ral::mt_size_t n_phototaxiing_to_goal_no_ca{0};
};

struct block_transporter_metrics_data : public rmetrics::base_data {
  block_transporter_metrics_data_impl interval{};
  block_transporter_metrics_data_impl cum{};

  /**
   * \brief Accumulate data. We ignore the "cum" field on \p rhs, and accumulate
   * into our "cum" field using the "interval" field of \p rhs.
   *
   * This is the most meaningful semantics I could come up with; I couldn't find
   * a way to justify accumulating already cumulative data again (it would have
   * required some additional changes/contortions elsewhere).
   */
  block_transporter_metrics_data& operator+=(const block_transporter_metrics_data &rhs) {
    ral::mt_accum(this->interval.n_phototaxiing_to_goal_including_ca,
                  rhs.interval.n_phototaxiing_to_goal_including_ca);
    ral::mt_accum(this->interval.n_phototaxiing_to_goal_no_ca,
                  rhs.interval.n_phototaxiing_to_goal_no_ca);

    ral::mt_accum(this->cum.n_phototaxiing_to_goal_including_ca,
                  rhs.interval.n_phototaxiing_to_goal_including_ca);
    ral::mt_accum(this->cum.n_phototaxiing_to_goal_no_ca,
                  rhs.interval.n_phototaxiing_to_goal_no_ca);
    return *this;
  }
};

} /* namespace cosm::fsm::metrics */
