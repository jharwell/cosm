/**
 * \file distributor_metrics_data.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_data.hpp"


/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, foraging, block_dist, metrics, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct distributor_metrics_data
 * \ingroup foraging block_dist metrics detail
 *
 * \brief Container for holding collected statistics of \ref
 * distributor_metrics.
 */
struct distributor_metrics_data  {
  /**
   * \brief  Total # blocks distributed in interval.
   */
  size_t n_configured_clusters{0};

  /**
   * \brief  Total # cube blocks distributed in interval.
   */
  size_t n_mapped_clusters{0};

  /**
   * \brief  Total # ramp blocks distributed in interval.
   */
  size_t capacity{0};

  /**
   * \brief Total # distributorers for distributed blocks in interval.
   */
  size_t size{0};
};

NS_END(detail);

struct distributor_metrics_data : public rmetrics::base_data {
  detail::distributor_metrics_data cum{};
  detail::distributor_metrics_data interval{};
};

NS_END(metrics, block_dist, foraging, cosm);

