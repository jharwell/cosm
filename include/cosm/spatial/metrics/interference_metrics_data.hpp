/**
 * \file interference_metrics_data.hpp
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

#ifndef INCLUDE_COSM_SPATIAL_METRICS_INTERFERENCE_METRICS_DATA_HPP_
#define INCLUDE_COSM_SPATIAL_METRICS_INTERFERENCE_METRICS_DATA_HPP_

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
};

NS_END(metrics, spatial, cosm);

#endif /* INCLUDE_COSM_SPATIAL_METRICS_INTERFERENCE_METRICS_DATA_HPP_ */
