/**
 * \file block_cluster_metrics.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_metrics.hpp"
#include "rcppsw/types/type_uuid.hpp"
#include "rcppsw/math/range.hpp"
#include "rcppsw/math/vector2.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::foraging::metrics {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_cluster_metrics
 * \ingroup foraging metrics
 *
 * \brief Defines the metrics to be collected from \ref cfrepr::block_cluster
 * objects.
 *
 * Metrics should be collected every timestep.
 */
class block_cluster_metrics : public rmetrics::base_metrics {
 public:
  block_cluster_metrics(void) = default;
  ~block_cluster_metrics(void) override = default;

  /**
   * \brief Return the UUID of the cluster.
   */
  virtual rtypes::type_uuid id(void) const = 0;

  /**
   * \brief Return the total # of blocks within the specified cluster this
   * timestep.
   */
  virtual size_t n_blocks(void) const = 0;

  /**
   * \brief The extent of the cluster in X (NOT the dimensions).
   */
  virtual rmath::ranged xrspan(void) const = 0;

  /**
   * \brief The extent of the cluster in Y (NOT the dimensions).
   */
  virtual rmath::ranged yrspan(void) const = 0;

  /**
   * \brief The real-valued anchor point of the custer.
   */
  virtual rmath::vector2d ranchor2D(void) const = 0;
};

} /* namespace cosm::foraging::metrics */

