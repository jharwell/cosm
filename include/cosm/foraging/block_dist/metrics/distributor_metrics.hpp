/**
 * \file distributor_metrics.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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
namespace cosm::foraging::block_dist::metrics {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class distributor_metrics
 * \ingroup foraging block_dist metrics
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

} /* namespace cosm::foraging::block_dist::metrics */

