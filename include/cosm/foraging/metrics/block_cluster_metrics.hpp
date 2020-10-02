/**
 * \file block_cluster_metrics.hpp
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

#ifndef INCLUDE_COSM_FORAGING_METRICS_BLOCK_CLUSTER_METRICS_HPP_
#define INCLUDE_COSM_FORAGING_METRICS_BLOCK_CLUSTER_METRICS_HPP_

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
NS_START(cosm, foraging, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_cluster_metrics
 * \ingroup cosm foraging metrics
 *
 * \brief Defines the metrics to be collected from \ref block_cluster objects.
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

NS_END(metrics, foraging, cosm);

#endif /* INCLUDE_COSM_FORAGING_METRICS_BLOCK_CLUSTER_METRICS_HPP_ */
