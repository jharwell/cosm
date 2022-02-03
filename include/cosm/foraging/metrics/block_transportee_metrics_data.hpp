/**
 * \file block_transportee_metrics_data.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_FORAGING_METRICS_BLOCK_TRANSPORTEE_METRICS_DATA_HPP_
#define INCLUDE_COSM_FORAGING_METRICS_BLOCK_TRANSPORTEE_METRICS_DATA_HPP_

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
};

NS_END(metrics, foraging, cosm);

#endif /* INCLUDE_COSM_FORAGING_METRICS_BLOCK_TRANSPORTEE_METRICS_DATA_HPP_ */
