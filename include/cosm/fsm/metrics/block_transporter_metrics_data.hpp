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

#ifndef INCLUDE_COSM_FSM_METRICS_BLOCK_TRANSPORTER_METRICS_DATA_HPP_
#define INCLUDE_COSM_FSM_METRICS_BLOCK_TRANSPORTER_METRICS_DATA_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <atomic>

#include "rcppsw/metrics/base_metrics_data.hpp"

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
  std::atomic_size_t n_phototaxiing_to_goal_including_ca{0};
  std::atomic_size_t n_phototaxiing_to_goal_no_ca{0};
};

NS_END(detail);

struct block_transporter_metrics_data : public rmetrics::base_metrics_data {
  detail::block_transporter_metrics_data interval{};
  detail::block_transporter_metrics_data cum{};
};

NS_END(metrics, fsm, cosm);

#endif /* INCLUDE_COSM_FSM_METRICS_BLOCK_TRANSPORTER_METRICS_DATA_HPP_ */
