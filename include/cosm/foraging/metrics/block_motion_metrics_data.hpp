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
NS_START(cosm, foraging, metrics, detail);

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

NS_END(detail);

struct block_motion_metrics_data : public rmetrics::base_data {
  detail::block_motion_metrics_data_impl interval{};
  detail::block_motion_metrics_data_impl cum{};
};

NS_END(metrics, foraging, cosm);

