/**
 * \file goal_acq_metrics_data.hpp
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
struct goal_acq_data_impl {
  ral::mt_size_t n_true_exploring_for_goal{0};
  ral::mt_size_t n_false_exploring_for_goal{0};
  ral::mt_size_t n_vectoring_to_goal{0};
  ral::mt_size_t n_acquiring_goal{0};
};

struct goal_acq_metrics_data : public rmetrics::base_data {
  goal_acq_data_impl interval{};
  goal_acq_data_impl cum{};
};

} /* namespace cosm::spatial::metrics */

