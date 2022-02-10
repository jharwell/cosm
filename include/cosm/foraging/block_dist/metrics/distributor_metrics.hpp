/**
 * \file distributor_metrics.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "rcppsw/metrics/base_metrics.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/types/timestep.hpp"

#include "cosm/repr/block_type.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, block_dist, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class distributor_metrics
 * \ingroup cosm foraging block_dist metrics
 *
 * \brief Defines the metrics to be collected from block distributors about
 * block distributor.
 *
 * Metrics should be collected every timestep.
 */
class distributor_metrics : public rmetrics::base_metrics {
 public:
  distributor_metrics(void) = default;

  /**
   * \brief Return the total # of block clusters as read from the initialization
   * file.
   */
  virtual size_t n_configured_clusters(void) const = 0;

  /**
   * \brief Return the total # of block clusters which were successfully mapped
   * into the area.
   */
  virtual size_t n_mapped_clusters(void) const = 0;

  /**
   * \brief Return the total capacity across all clusters managed by the
   * distributor.
   */
  virtual size_t capacity(void) const = 0;

  /**
   * \brief Return the total size (i.e. the # blocks currently contained) across
   * all clusters managed by the distributor.
   */
  virtual size_t size(void) const = 0;
};

NS_END(metrics, block_dist, foraging, cosm);

