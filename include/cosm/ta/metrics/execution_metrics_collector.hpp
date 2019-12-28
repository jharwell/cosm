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
#include <list>
#include <string>
#include <atomic>

#include "rcppsw/metrics/base_metrics_collector.hpp"
#include "rcppsw/er/client.hpp"
#include "cosm/ta/time_estimate.hpp"

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
 */
class execution_metrics_collector final : public rmetrics::base_metrics_collector,
                                          public rer::client<execution_metrics_collector> {
 public:
  /**
   * \param ofname Output file name.
   * \param interval Collection interval.
   */
  execution_metrics_collector(const std::string& ofname, uint interval);

  void reset(void) override;
  void collect(const rmetrics::base_metrics& metrics) override;
  void reset_after_interval(void) override;

  std::list<std::string> csv_header_cols(void) const override;
  boost::optional<std::string> csv_line_build(void) override;

 private:
  struct stats {
    /**
     * \brief # Times the task has been completed.
     */
    std::atomic_uint complete_count{0};

    /**
     * \brief # Times the task has been aborted.
     */
    std::atomic_uint abort_count{0};

    /**
     * \brief # Times at their interface.
     */
    std::atomic_uint interface_count{0};

    /**
     * \brief Execution times of the task.
     */
    std::atomic_uint exec_time{};

    /**
     * \brief Interface time of the task.
     */
    std::atomic_uint interface_time{0};

    std::atomic_uint exec_estimate{0};
    std::atomic_uint interface_estimate{0};
  };

  /* clang-format off */
  struct stats m_interval{};
  struct stats m_cum{};

  /* clang-format on */
};

NS_END(metrics, ta, cosm);

#endif /* INCLUDE_COSM_TA_METRICS_EXECUTION_METRICS_COLLECTOR_HPP_ */
