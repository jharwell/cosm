/**
 * \file acq_metrics_collector.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <list>

#include "rcppsw/metrics/base_collector.hpp"

#include "cosm/spatial/strategy/nest/metrics/acq_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, spatial, strategy, nest, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class acq_metrics_collector
 * \ingroup spatial strategy nest metrics
 *
 * \brief Collector for \ref acq_metrics.
 *
 * Metrics CAN be collected in parallel from robots; concurrent updates to the
 * gathered stats are supported. Metrics are written out at the end of the
 * specified interval.
 */
class acq_metrics_collector final : public rmetrics::base_collector {
 public:
    /**
   * \param sink The metrics sink to use.
   */
  explicit acq_metrics_collector(
      std::unique_ptr<rmetrics::base_sink> sink);

  /* base_collector overrides */
  void collect(const rmetrics::base_metrics& metrics) override;
  void reset_after_interval(void) override;
  const rmetrics::base_data* data(void) const override { return &m_data; }

 private:
  /* clang-format off */
  acq_metrics_data m_data{};
  /* clang-format on */
};

NS_END(metrics, nest, strategy, spatial, cosm);