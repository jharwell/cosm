/**
 * \file nest_zone_metrics_data.hpp
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
namespace cosm::spatial::metrics {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \brief Container for holding collected statistics. Must be atomic so counts
 * are valid in parallel metric collection contexts.
 */
struct nest_zone_metrics_data_impl {
  ral::mt_size_t n_in_nest{0};
  ral::mt_size_t n_entered_nest{0};
  ral::mt_size_t n_exited_nest{0};
  ral::mt_size_t nest_duration{0};
  ral::mt_size_t first_nest_entry_time{0};
};

struct nest_zone_metrics_data : public rmetrics::base_data {
  nest_zone_metrics_data_impl interval{};
  nest_zone_metrics_data_impl cum{};
};

} /* namespace cosm::spatial::metrics */
