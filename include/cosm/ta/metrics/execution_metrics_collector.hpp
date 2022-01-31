/**
 * \file execution_metrics_collector.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_TA_METRICS_EXECUTION_METRICS_COLLECTOR_HPP_
#define INCLUDE_COSM_TA_METRICS_EXECUTION_METRICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/metrics/base_collector.hpp"
#include "rcppsw/er/client.hpp"

#include "cosm/ta/metrics/execution_metrics_data.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class execution_metrics_collector
 * \ingroup ta metrics
 *
 * \brief Collector for metrics for an executable task across executions of that
 * task.
 *
 * Metrics CAN be collected in parallel from robots; concurrent updates to the
 * gathered stats are supported. Metrics are output at the specified interval
 */
class execution_metrics_collector final : public rmetrics::base_collector,
                                          public rer::client<execution_metrics_collector> {
 public:
  /**
   * \param sink The metrics sink to use.
   */
  explicit execution_metrics_collector(
      std::unique_ptr<rmetrics::base_sink> sink);

  /* base_collector overrides */
  void collect(const rmetrics::base_metrics& metrics) override;
  void reset_after_interval(void) override;
  const rmetrics::base_data* data(void) const override { return &m_data; }

 private:
  /* clang-format off */
  execution_metrics_data m_data{};
  /* clang-format on */
};

NS_END(metrics, ta, cosm);

#endif /* INCLUDE_COSM_TA_METRICS_EXECUTION_METRICS_COLLECTOR_HPP_ */
