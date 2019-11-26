/**
 * \file transport_metrics.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_METRICS_BLOCKS_TRANSPORT_METRICS_HPP_
#define INCLUDE_COSM_METRICS_BLOCKS_TRANSPORT_METRICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_metrics.hpp"
#include "cosm/repr/block_type.hpp"
#include "rcppsw/types/timestep.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, metrics, blocks);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class transport_metrics
 * \ingroup cosm metrics blocks
 *
 * \brief Defines the metrics to be collected from blocks about the process of
 * transportation from their original location in the arena after distribution
 * to their final destination (nest, structure, etc).
 *
 * Metrics should be collected upon deposition at the block's final location,
 * rather than every timestep.
 */
class transport_metrics : public rmetrics::base_metrics {
 public:
  transport_metrics(void) = default;

  /**
   * \brief Return the total # of robots that have carried the block since it
   * was originally distributed in the arena until it arrives at its final
   * destination.
   */
  virtual uint total_transporters(void) const = 0;

  /**
   * \brief Return the total amount of time that it took from the first pickup
   * to when the block was deposited at its final destination.
   */
  virtual rtypes::timestep total_transport_time(void) const = 0;

  /**
   * \brief Return the amount of time that the block sits in the arena after
   * being distributed but before it is picked up for the first time.
   */
  virtual rtypes::timestep initial_wait_time(void) const = 0;

  /**
   * \brief Return the type of the block transported.
   */
  virtual repr::block_type type(void) const = 0;
};

NS_END(blocks, metrics, cosm);

#endif /* INCLUDE_COSM_METRICS_BLOCKS_TRANSPORT_METRICS_HPP_ */
