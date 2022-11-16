/**
 * \file interference_locs3D_metrics_collector.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/ds/metrics/grid3D_metrics_collector.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, spatial, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class interference_locs3D_metrics_collector
 * \ingroup spatial metrics
 *
 * \brief Collector for \ref interference_metrics as a 3D grid of where robots
 * most frequently encounter other robots.
 *
 * Metrics CAN be collected concurrently if the calling context guarantees that
 * no two robots will have the same discrete location. Otherwise, serial
 * collection is required.
 */
class interference_locs3D_metrics_collector final : public rdmetrics::grid3D_metrics_collector {
 public:
  /**
   * \param sink The metrics sink to use.
   * \param dims Dimensions of arena.
   */
  interference_locs3D_metrics_collector(
      std::unique_ptr<rmetrics::base_sink> sink,
      const rmath::vector3z& dims);

  void collect(const rmetrics::base_metrics& metrics) override;
};

NS_END(metrics, spatial, cosm);

