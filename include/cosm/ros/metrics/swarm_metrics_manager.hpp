/**
 * \file swarm_metrics_manager.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <map>
#include <string>
#include <ros/ros.h>

#include "rcppsw/er/client.hpp"

#include "rcppsw/metrics/config/metrics_config.hpp"
#include "rcppsw/metrics/fs_output_manager.hpp"

#include "cosm/cosm.hpp"
#include "cosm/ros/topic.hpp"
#include "cosm/ros/spatial/metrics/movement_metrics_msg.hpp"
#include "cosm/ros/spatial/metrics/interference_metrics_msg.hpp"
#include "cosm/ros/fsm/metrics/block_transporter_metrics_msg.hpp"
#include "cosm/ros/foraging/metrics/block_transportee_metrics_msg.hpp"
#include "cosm/ros/foraging/metrics/block_cluster_metrics_msg.hpp"
#include "cosm/hal/ros/sensors/metrics/battery_metrics_msg.hpp"
#include "cosm/ros/metrics/msg_tracking_map.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::ros::metrics {
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

  /* fs_output_manager overrides */
  bool flush(const rmetrics::output_mode& mode,
             const rtypes::timestep&) override;

  /**
   * \brief We can't reset metric collectors according to the current timestep,
   * because we may receive packets from robots asynchronously. Instead, we
   * maintain internal state and flush the ones that are ready each timestep.
   */
  void interval_reset(const rtypes::timestep&) override;

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
   * - \ref cmspecs::sensors::kBattery -> \ref
   *   chsensors::metrics::battery_metrics
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

  /**
   * After subscribing to a metric data topic, verify connection.
   */
  bool wait_for_connection(const ::ros::Subscriber& sub);

  msg_tracking_map* msg_tracking(void) { return &m_tracking; }

 private:
  void collect(const boost::shared_ptr<const crsmetrics::movement_metrics_msg>& msg);
  void collect(const boost::shared_ptr<const crsmetrics::interference_metrics_msg>& msg);
  void collect(const boost::shared_ptr<const crfsm::metrics::block_transporter_metrics_msg>& msg);
  void collect(const boost::shared_ptr<const crfmetrics::block_transportee_metrics_msg>& msg);
  void collect(const boost::shared_ptr<const chros::sensors::metrics::battery_metrics_msg>& msg);
  void collect(const boost::shared_ptr<const crfmetrics::block_cluster_metrics_msg>& msg);


  /* clang-format off */
  /**
   * \brief Maps an output mode to the expected # of metric packets that should
   * be received via subscription during a given interval.
   */
  using expected_counts_map_type = std::map<rmetrics::output_mode, size_t>;

  const expected_counts_map_type mc_expected_counts{};

  msg_tracking_map               m_tracking{};
  std::vector<::ros::Subscriber> m_subs{};
  /* clang-format on */
};

} /* namespace cosm::ros::metrics */
