/**
 * \file population_dynamics_metrics_data.hpp
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
namespace cosm::tv::metrics {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \brief Container for holding population dynamics statistics collected from
 * the swarm. Must be atomic so counts are valid in parallel metric collection
 * contexts.
 */
struct population_dynamics_metrics_data_impl {
  /* clang-format off */
  ral::mt_size_t  n_births{0};
  ral::mt_size_t  birth_interval{0};
  ral::mt_double_t birth_mu{0};

  ral::mt_size_t  n_deaths{0};
  ral::mt_size_t  death_interval{0};
  ral::mt_double_t death_lambda{0};

  ral::mt_size_t  repair_queue_size{0};
  ral::mt_size_t  n_malfunctions{0};
  ral::mt_size_t  malfunction_interval{0};
  ral::mt_double_t malfunction_lambda{0};

  ral::mt_size_t  n_repairs{0};
  ral::mt_size_t  repair_interval{0};
  ral::mt_double_t repair_mu{0};

  ral::mt_size_t  total_population{0};
  ral::mt_size_t  active_population{0};
  ral::mt_size_t  max_population{0};
  /* clang-format on */
};

struct population_dynamics_metrics_data : public rmetrics::base_data {
  population_dynamics_metrics_data_impl interval{};
  population_dynamics_metrics_data_impl cum{};
};

} /* namespace cosm::tv::metrics */

