/**
 * \file block_cluster_metrics_collector.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/foraging/metrics/block_cluster_metrics_collector.hpp"

#include "cosm/foraging/metrics/block_cluster_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_cluster_metrics_collector::block_cluster_metrics_collector(
    const std::string& ofname_stem,
    const rtypes::timestep& interval,
    size_t n_clusters)
    : base_metrics_collector(ofname_stem,
                             interval,
                             rmetrics::output_mode::ekAPPEND),
      m_int_block_counts(n_clusters),
      m_cum_block_counts(n_clusters),
      m_extents(n_clusters) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string> block_cluster_metrics_collector::csv_header_cols(void) const {
  auto cols = dflt_csv_header_cols();

  for (size_t i = 0; i < m_int_block_counts.size(); ++i) {
    cols.push_back("int_avg_cluster" + rcppsw::to_string(i) + "_block_count");
  } /* for(i..) */

  for (size_t i = 0; i < m_cum_block_counts.size(); ++i) {
    cols.push_back("cum_avg_cluster" + rcppsw::to_string(i) + "_block_count");
  } /* for(i..) */

  for (size_t i = 0; i < m_extents.size(); ++i) {
    cols.push_back("cluster" + rcppsw::to_string(i) + "_area");
    cols.push_back("cluster" + rcppsw::to_string(i) + "_xmin");
    cols.push_back("cluster" + rcppsw::to_string(i) + "_xmax");
    cols.push_back("cluster" + rcppsw::to_string(i) + "_ymin");
    cols.push_back("cluster" + rcppsw::to_string(i) + "_ymax");
  } /* for(i..) */

  return cols;
} /* csv_header_cols() */

void block_cluster_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

boost::optional<std::string> block_cluster_metrics_collector::csv_line_build(void) {
  if (!(timestep() % interval() == 0)) {
    return boost::none;
  }
  std::string line;

  for (auto &count : m_int_block_counts) {
    line += csv_entry_intavg(count);
  } /* for(&count..) */

  for (auto &count : m_cum_block_counts) {
    line += csv_entry_tsavg(count);
  } /* for(&count..) */

  for (size_t i = 0; i < m_extents.size() - 1; ++i) {
    line += rcppsw::to_string(m_extents[i].area) + separator();
    line += rcppsw::to_string(m_extents[i].xmin) + separator();
    line += rcppsw::to_string(m_extents[i].xmax) + separator();
    line += rcppsw::to_string(m_extents[i].ymin) + separator();
    line += rcppsw::to_string(m_extents[i].ymax) + separator();
  } /* for(i..) */

  line += rcppsw::to_string(m_extents[m_extents.size() - 1].area) + separator();
  line += rcppsw::to_string(m_extents[m_extents.size() - 1].xmin) + separator();
  line += rcppsw::to_string(m_extents[m_extents.size() - 1].xmax) + separator();
  line += rcppsw::to_string(m_extents[m_extents.size() - 1].ymin) + separator();
  line += rcppsw::to_string(m_extents[m_extents.size() - 1].ymax);

  return boost::make_optional(line);
} /* csv_line_build() */

void block_cluster_metrics_collector::collect(const rmetrics::base_metrics& metrics) {
  auto& m = static_cast<const block_cluster_metrics&>(metrics);
  m_int_block_counts[m.id().v()] += m.n_blocks();
  m_cum_block_counts[m.id().v()] += m.n_blocks();

  auto xmin = m_extents[m.id().v()].xmin.load();
  auto xmax = m_extents[m.id().v()].xmax.load();
  auto ymin = m_extents[m.id().v()].ymin.load();
  auto ymax = m_extents[m.id().v()].ymax.load();
  auto area = m_extents[m.id().v()].area.load();

  m_extents[m.id().v()].xmin.compare_exchange_strong(xmin,
                                                     m.ranchor2D().x());
  m_extents[m.id().v()].xmax.compare_exchange_strong(xmax,
                                                     m.ranchor2D().x() + m.xrspan().span());

  m_extents[m.id().v()].ymin.compare_exchange_strong(ymin,
                                                     m.ranchor2D().y());
  m_extents[m.id().v()].ymax.compare_exchange_strong(ymax,
                                                     m.ranchor2D().y() + m.yrspan().span());
  m_extents[m.id().v()].area.compare_exchange_strong(area,
                                                     m.xrspan().span() * m.yrspan().span());
} /* collect() */

void block_cluster_metrics_collector::reset_after_interval(void) {
  for (size_t i = 0; i < m_int_block_counts.size(); ++i) {
    std::atomic_init(&m_int_block_counts[i], 0U);
  } /* for(i..) */
} /* reset_after_interval() */

NS_END(metrics, foraging, cosm);
