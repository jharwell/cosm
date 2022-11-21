/**
 * \file block_motion_metrics_data.hpp
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
#include "rcppsw/al/multithread.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::foraging::metrics {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct block_motion_metrics_data_impl
 * \ingroup foraging block_dist metrics detail
 *
 * \brief Container for holding collected statistics of \ref
 * block_motion_metrics.
 */
struct block_motion_metrics_data_impl  {
  /**
   * \brief  Total # blocks moved.
   */
  ral::mt_size_t n_moved{0};
};

struct block_motion_metrics_data : public rmetrics::base_data {
  block_motion_metrics_data_impl interval{};
  block_motion_metrics_data_impl cum{};
};

} /* namespace cosm::foraging::metrics */

