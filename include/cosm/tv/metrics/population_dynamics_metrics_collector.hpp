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

#include "rcppsw/metrics/base_metrics_collector.hpp"

#include "cosm/tv/metrics/population_dynamics_metrics_data.hpp"

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
 * Metrics CAN be collected in parallel; concurrent updates to the gathered
 * stats are supported.
 */
class population_dynamics_metrics_collector final : public rmetrics::base_metrics_collector {
 public:
  /**
   * \param sink The metrics sink to use.
   */
  explicit population_dynamics_metrics_collector(
      std::unique_ptr<rmetrics::base_metrics_sink> sink);

  /* base_metrics_collector overrides */
  void collect(const rmetrics::base_metrics& metrics) override;
  void reset_after_interval(void) override;
  const rmetrics::base_metrics_data* data(void) const override { return &m_data; }


 private:
  /* clang-format off */
  population_dynamics_metrics_data m_data{};
};

NS_END(metrics, tv, cosm);

#endif /* INCLUDE_COSM_TV_METRICS_POPULATION_DYNAMICS_METRICS_COLLECTOR_HPP_ */
