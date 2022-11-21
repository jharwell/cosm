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
namespace cosm::foraging::block_dist::metrics {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct distributor_metrics_data_impl
 * \ingroup foraging block_dist metrics detail
 *
 * \brief Container for holding collected statistics of \ref
 * distributor_metrics.
 */
struct distributor_metrics_data_impl  {
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

struct distributor_metrics_data : public rmetrics::base_data {
  distributor_metrics_data_impl cum{};
  distributor_metrics_data_impl interval{};
};

} /* namespace cosm::foraging::block_dist::metrics */

