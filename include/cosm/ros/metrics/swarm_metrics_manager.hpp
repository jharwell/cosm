/**
 * \file swarm_metrics_manager.hpp
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

#ifndef INCLUDE_COSM_ROS_METRICS_SWARM_METRICS_MANAGER_HPP_
#define INCLUDE_COSM_ROS_METRICS_SWARM_METRICS_MANAGER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <ros/ros.h>

#include "rcppsw/er/client.hpp"

#include "rcppsw/metrics/config/metrics_config.hpp"
#include "rcppsw/metrics/fs_output_manager.hpp"

#include "cosm/cosm.hpp"
#include "cosm/ros/topic.hpp"

#include "cosm/spatial/metrics/movement_metrics_data.hpp"
#include "cosm/spatial/metrics/interference_metrics_data.hpp"
#include "cosm/fsm/metrics/block_transporter_metrics_data.hpp"
#include "cosm/foraging/metrics/block_transportee_metrics_data.hpp"
#include "cosm/foraging/metrics/block_cluster_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, ros, metrics);
namespace fs = std::filesystem;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class swarm_metrics_manager
 * \ingroup ros metrics
 *
 * \brief Manager class for handling all of the metrics which can be generated
 * by COSM on robots running ROS. Runs on the ROS master to gather metrics from
 * all robots and write them out to the file system.
 *
 * To transfer custom classes via ROS messages, you can't have any std::atomic
 * bits in the classes, because they aren't copyable or movable, which is
 * needed. But that's OK because ROS (apparently) calls all subscribed callbacks
 * serially rather than in threads, so there is no possibility of race
 * conditions.
 */
class swarm_metrics_manager : public rer::client<swarm_metrics_manager>,
                              public rmetrics::fs_output_manager {
 public:
  static constexpr const size_t kQueueBufferSize = 1000;

  swarm_metrics_manager(const rmconfig::metrics_config* mconfig,
                        const fs::path& root,
                        size_t n_robots);
  ~swarm_metrics_manager(void) override = default;

 protected:
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
  void register_standard(const rmconfig::metrics_config* mconfig,
                         size_t n_robots);

  /**
   * \brief Register metrics collectors that require the # of block clusters in
   * the arena.
   *
   * - \ref cmspecs::blocks::kClusters -> \ref cfmetrics::block_cluster_metrics
   */
  void register_with_n_block_clusters(const rmconfig::metrics_config* mconfig,
                                      size_t n_robots,
                                      size_t n_block_clusters);

 private:
  void collect(const boost::shared_ptr<const csmetrics::movement_metrics_data>& data);
  void collect(const boost::shared_ptr<const csmetrics::interference_metrics_data>& data);
  void collect(const boost::shared_ptr<const cfsm::metrics::block_transporter_metrics_data>& data);
  void collect(const boost::shared_ptr<const cforaging::metrics::block_transportee_metrics_data>& data);
  void collect(const boost::shared_ptr<const cforaging::metrics::block_cluster_metrics_data>& data);

  /* clang-format off */
  std::vector<::ros::Subscriber> m_subs{};
  /* clang-format on */
};

NS_END(metrics, ros, cosm);

#endif /* INCLUDE_COSM_ROS_METRICS_SWARM_METRICS_MANAGER_HPP_ */
