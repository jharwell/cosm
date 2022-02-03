/**
 * \file robot_output_manager.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_ROS_METRICS_ROBOT_METRICS_MANAGER_HPP_
#define INCLUDE_COSM_ROS_METRICS_ROBOT_METRICS_MANAGER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/er/client.hpp"

#include "rcppsw/metrics/config/metrics_config.hpp"
#include "rcppsw/metrics/network_output_manager.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {
class base_block3D;
} /* namespace cosm::repr */

namespace cosm::controller {
class base_controller2D;
class base_controllerQ3D;
} /* namespace cosm::controller */

NS_START(cosm, ros, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class robot_metrics_manager
 * \ingroup ros metrics
 *
 * \brief Manager class for handling all of the metrics which can be generated
 * by COSM on robots running ROS.
 */
class robot_metrics_manager : public rmetrics::network_output_manager,
                             public rer::client<robot_metrics_manager> {
 public:
  explicit robot_metrics_manager(const rmconfig::metrics_config* mconfig);
  ~robot_metrics_manager(void) override = default;

  /**
   * \brief Collect metrics from a 3D block right before it is dropped in the
   * nest. Currently this includes:
   *
   * - \ref cmspecs::blocks::kTransportee
   */
  void collect_from_block(const crepr::base_block3D* block);

  /**
   * \brief Collect metrics from 2D controllers. Currently this includes:
   *
   * - \ref cmspecs::spatial::kMovement
   * - \ref cmspecs::spatial::kInterferenceCounts
   */
  void collect_from_controller(const ccontroller::base_controller2D* controller);

 protected:
  /**
   * \brief Register metrics collectors that require the # of block clusters in
   * the arena.
   *
   * - \ref cmspecs::blocks::kClusters -> \ref cfmetrics::block_cluster_metrics
   */
  void register_with_n_block_clusters(const rmconfig::metrics_config* mconfig,
                                      size_t n_block_clusters);
  /**
   * \brief Register metrics collectors that do not require extra arguments.
   *
   * - \ref cmspecs::spatial::kMovement -> \ref csmetrics::movement_metrics
   * - \ref cmspecs::spatial::kInterferenceCounts -> \ref
   *   csmetrics::interference_metrics
   * - \ref cmspecs::blocks::kTransporter -> \ref
   *   cfsm::metrics::block_transporter_metrics
   * - \ref cmspecs::blocks::kTransportee -> \ref
   *  cfmetrics::block_transportee_metrics
   * - \ref cmspecs::blocks::kAcqCounts -> \ref csmetrics::goal_acq_metrics
   */
  void register_standard(const rmconfig::metrics_config* mconfig);
};

NS_END(metrics, ros, cosm);

#endif /* INCLUDE_COSM_ROS_METRICS_ROBOT_METRICS_MANAGER_HPP_ */
