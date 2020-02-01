/**
 * \file population_dynamics_metrics_collector.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_TV_METRICS_POPULATION_DYNAMICS_METRICS_COLLECTOR_HPP_
#define INCLUDE_COSM_TV_METRICS_POPULATION_DYNAMICS_METRICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <list>
#include <atomic>

#include "rcppsw/metrics/base_metrics_collector.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, tv, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class population_dynamics_metrics_collector
 * \ingroup cosm metrics tv
 *
 * \brief Collector for \ref population_dynamics_metrics.
 *
 * Metrics are written out at the specified collection interval.
 */
class population_dynamics_metrics_collector final : public rmetrics::base_metrics_collector {
 public:
  /**
   * \param ofname The output file name.
   * \param interval Collection interval.
   */
  population_dynamics_metrics_collector(const std::string& ofname,
                                        uint interval);

  void reset(void) override;
  void collect(const rmetrics::base_metrics& metrics) override;
  void reset_after_interval(void) override;

 private:
  /**
   * \brief Container for holding population dynamics statistics collected from
   * the swarm. Must be atomic so counts are valid in parallel metric collection
   * contexts.
   */
  struct stats {
    /* clang-format off */
    std::atomic_uint    n_births{0};
    std::atomic_uint    birth_interval{0};
    std::atomic<double> birth_mu{0};

    std::atomic_uint    n_deaths{0};
    std::atomic_uint    death_interval{0};
    std::atomic<double> death_lambda{0};

    std::atomic_uint    repair_queue_size{0};
    std::atomic_uint    n_malfunctions{0};
    std::atomic_uint    malfunction_interval{0};
    std::atomic<double> malfunction_lambda{0};

    std::atomic_uint    n_repairs{0};
    std::atomic_uint    repair_interval{0};
    std::atomic<double> repair_mu{0};

    std::atomic_uint    swarm_population{0};
    std::atomic_uint    swarm_max_population{0};
    /* clang-format on */
  };


  std::list<std::string> csv_header_cols(void) const override;
  boost::optional<std::string> csv_line_build(void) override;

  /* clang-format off */
  struct stats m_interval{};
  struct stats m_cum{};
  /* clang-format on */
};

NS_END(metrics, tv, cosm);

#endif /* INCLUDE_COSM_TV_METRICS_POPULATION_DYNAMICS_METRICS_COLLECTOR_HPP_ */
