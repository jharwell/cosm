/**
 * \file population_dynamics_metrics_data.hpp
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

#ifndef INCLUDE_COSM_TV_METRICS_POPULATION_DYNAMICS_METRICS_DATA_HPP_
#define INCLUDE_COSM_TV_METRICS_POPULATION_DYNAMICS_METRICS_DATA_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <atomic>

#include "rcppsw/metrics/base_metrics_data.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, tv, metrics, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \brief Container for holding population dynamics statistics collected from
 * the swarm. Must be atomic so counts are valid in parallel metric collection
 * contexts.
 */
struct population_dynamics_metrics_data {
  /* clang-format off */
  std::atomic_size_t  n_births{0};
  std::atomic_size_t  birth_interval{0};
  std::atomic<double> birth_mu{0};

  std::atomic_size_t  n_deaths{0};
  std::atomic_size_t  death_interval{0};
  std::atomic<double> death_lambda{0};

  std::atomic_size_t  repair_queue_size{0};
  std::atomic_size_t  n_malfunctions{0};
  std::atomic_size_t  malfunction_interval{0};
  std::atomic<double> malfunction_lambda{0};

  std::atomic_size_t  n_repairs{0};
  std::atomic_size_t  repair_interval{0};
  std::atomic<double> repair_mu{0};

  std::atomic_size_t  total_population{0};
  std::atomic_size_t  active_population{0};
  std::atomic_size_t  max_population{0};
  /* clang-format on */
};


NS_END(detail);

struct population_dynamics_metrics_data : public rmetrics::base_metrics_data {
  detail::population_dynamics_metrics_data interval{};
  detail::population_dynamics_metrics_data cum{};
};

NS_END(metrics, tv, cosm);

#endif /* INCLUDE_COSM_TV_METRICS_POPULATION_DYNAMICS_METRICS_DATA_HPP_ */
