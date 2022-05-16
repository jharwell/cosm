/**
 * \file block_transporter_metrics_data.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_data.hpp"
#include "rcppsw/al/multithread.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, fsm, metrics, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct block_transporter_metrics_data
 * \ingroup fsm metrics detail
 *
 * \brief Container for holding \ref block_transporter_metrics data. Must
 * be atomic so counts are valid in parallel metric collection contexts.
 */
struct block_transporter_metrics_data {
  ral::mt_size_t n_phototaxiing_to_goal_including_ca{0};
  ral::mt_size_t n_phototaxiing_to_goal_no_ca{0};
};

NS_END(detail);

struct block_transporter_metrics_data : public rmetrics::base_data {
  detail::block_transporter_metrics_data interval{};
  detail::block_transporter_metrics_data cum{};

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

NS_END(metrics, fsm, cosm);
