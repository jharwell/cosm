/**
 * \file block_transportee_metrics_data.hpp
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
 * \brief Container for holding \ref block_transportee_metrics data. Must use
 * atomics so counts are valid in parallel metric collection contexts. Ideally
 * the times would be atomic \ref rtypes::timestep, but that type does not meet
 * the std::atomic requirements.
 */
struct block_transportee_metrics_data {
  /**
   * \brief  Total # blocks transported.
   */
  ral::mt_size_t n_transported{0};

  /**
   * \brief  Total # cube blocks transported.
   */
  ral::mt_size_t n_cube_transported{0};

  /**
   * \brief  Total # ramp blocks transported in interval.
   */
  ral::mt_size_t n_ramp_transported{0};

  /**
   * \brief Total # transporters for transported blocks in interval.
   */
  ral::mt_size_t n_transporters{0};

  /**
   * \brief Total amount of time taken for all transported blocks to be
   * transported from original distribution locations to the nest within an
   * interval.
   */
  ral::mt_size_t transport_time{0};

  /**
   * \brief Total amount of time between original arena distribution and first
   * pickup for all transported blocks in interval.
   */
  ral::mt_size_t initial_wait_time{0};
};

NS_END(detail);

struct block_transportee_metrics_data : public rmetrics::base_data {
  detail::block_transportee_metrics_data interval{};
  detail::block_transportee_metrics_data cum{};

  /**
   * \brief Accumulate data. We ignore the "cum" field on \p rhs, and accumulate
   * into our "cum" field using the "interval" field of \p rhs.
   *
   * This is the most meaningful semantics I could come up with; I couldn't find
   * a way to justify accumulating already cumulative data again (it would have
   * required some additional changes/contortions elsewhere).
   */
  block_transportee_metrics_data& operator+=(const  block_transportee_metrics_data& rhs) {
    ral::mt_accum(this->interval.n_transported,
                  rhs.interval.n_transported);
    ral::mt_accum(this->interval.n_cube_transported,
                  rhs.interval.n_cube_transported);
    ral::mt_accum(this->interval.n_ramp_transported,
                  rhs.interval.n_ramp_transported);
    ral::mt_accum(this->interval.n_transporters,
                  rhs.interval.n_transporters);
    ral::mt_accum(this->interval.transport_time,
                  rhs.interval.transport_time);
    ral::mt_accum(this->interval.initial_wait_time,
                  rhs.interval.initial_wait_time);

    ral::mt_accum(this->cum.n_transported,
                  rhs.interval.n_transported);
    ral::mt_accum(this->cum.n_cube_transported,
                  rhs.interval.n_cube_transported);
    ral::mt_accum(this->cum.n_ramp_transported,
                  rhs.interval.n_ramp_transported);
    ral::mt_accum(this->cum.n_transporters,
                  rhs.interval.n_transporters);
    ral::mt_accum(this->cum.transport_time,
                  rhs.interval.transport_time);
    ral::mt_accum(this->cum.initial_wait_time,
                  rhs.interval.initial_wait_time);

    return *this;
  }
};

NS_END(metrics, foraging, cosm);
