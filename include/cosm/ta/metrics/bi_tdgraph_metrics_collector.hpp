/**
 * \file bi_tdgraph_metrics_collector.hpp
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

#ifndef INCLUDE_COSM_TA_METRICS_BI_TDGRAPH_METRICS_COLLECTOR_HPP_
#define INCLUDE_COSM_TA_METRICS_BI_TDGRAPH_METRICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <list>
#include <string>
#include <atomic>

#include "rcppsw/metrics/base_metrics_collector.hpp"
#include "rcppsw/rcppsw.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class bi_tdgraph_metrics_collector
 * \ingroup ta metrics
 *
 * \brief Collector for metrics about the current task_dist of tasks in a
 * collection of agents executing tasks in a \ref bi_tdgraph.
 *
 * Metrics CAN be collected in parallel from robots; concurrent updates to the
 * gathered stats are supported. Metrics are written out at the specified
 * interval.
 */
class bi_tdgraph_metrics_collector final : public rmetrics::base_metrics_collector {
 public:
  /**
   * \param ofname Output file name.
   * \param interval Collection interval.
   * \param decomposition_depth The maximum depth of the \ref bi_tdgraph.
   */
  bi_tdgraph_metrics_collector(const std::string& ofname,
                               const rtypes::timestep& interval,
                               uint decomposition_depth);

  void reset(void) override;
  void collect(const rmetrics::base_metrics& metrics) override;
  void reset_after_interval(void) override;

  std::list<std::string> csv_header_cols(void) const override;
  boost::optional<std::string> csv_line_build(void) override;

  const std::vector<std::atomic_uint>& int_task_counts(void) const {
    return m_int_task_counts;
  }

 private:
  /*
   * These are not in a struct because I need to be able to initialize the
   * vectors directly with the constructor arguments (well I COULD do it another
   * way, but that smells...).
   */
  /* clang-format off */
  std::vector<std::atomic_uint> m_int_depth_counts;
  std::vector<std::atomic_uint> m_int_task_counts;
  std::vector<std::atomic_uint> m_int_tab_counts;
  std::vector<std::atomic_uint> m_cum_depth_counts;
  std::vector<std::atomic_uint> m_cum_task_counts;
  std::vector<std::atomic_uint> m_cum_tab_counts;
  /* clang-format on */
};

NS_END(metrics, ta, cosm);


#endif /* INCLUDE_COSM_TA_METRICS_BI_TDGRAPH_METRICS_COLLECTOR_HPP_ */
