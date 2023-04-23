/**
 * \file swarm_metrics_manager.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ros/metrics/swarm_metrics_manager.hpp"

#include <boost/mpl/for_each.hpp>
#include <boost/range/adaptor/map.hpp>

#include "rcppsw/metrics/file_sink_registerer.hpp"
#include "rcppsw/metrics/register_using_config.hpp"
#include "rcppsw/metrics/register_with_sink.hpp"
#include "rcppsw/mpl/identity.hpp"
#include "rcppsw/mpl/typelist.hpp"

#include "cosm/controller/base_controller2D.hpp"
#include "cosm/foraging/metrics/block_cluster_metrics_collector.hpp"
#include "cosm/foraging/metrics/block_cluster_metrics_csv_sink.hpp"
#include "cosm/foraging/metrics/block_transportee_metrics_collector.hpp"
#include "cosm/foraging/metrics/block_transportee_metrics_csv_sink.hpp"
#include "cosm/fsm/metrics/block_transporter_metrics_collector.hpp"
#include "cosm/fsm/metrics/block_transporter_metrics_csv_sink.hpp"
#include "cosm/metrics/specs.hpp"
#include "cosm/pal/ros/swarm_iterator.hpp"
#include "cosm/ros/foraging/metrics/block_cluster_metrics_glue.hpp"
#include "cosm/ros/foraging/metrics/block_transportee_metrics_glue.hpp"
#include "cosm/ros/fsm/metrics/block_transporter_metrics_glue.hpp"
#include "cosm/ros/metrics/library.hpp"
#include "cosm/ros/spatial/metrics/interference_metrics_glue.hpp"
#include "cosm/ros/kin/metrics/kinematics_metrics_glue.hpp"
#include "cosm/spatial/metrics/interference_metrics_collector.hpp"
#include "cosm/spatial/metrics/interference_metrics_csv_sink.hpp"
#include "cosm/kin/metrics/kinematics_metrics.hpp"
#include "cosm/kin/metrics/kinematics_metrics_collector.hpp"
#include "cosm/kin/metrics/kinematics_metrics_dist_csv_sink.hpp"
#include "cosm/kin/metrics/kinematics_metrics_avg_csv_sink.hpp"
#include "cosm/hal/sensors/metrics/battery_metrics_csv_sink.hpp"
#include "cosm/hal/sensors/metrics/battery_metrics_collector.hpp"
#include "cosm/hal/ros/sensors/metrics/battery_metrics_glue.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::ros::metrics {

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
swarm_metrics_manager::swarm_metrics_manager(
    const rmconfig::metrics_config* const mconfig,
    const fs::path& output_root,
    size_t n_robots)
    : ER_CLIENT_INIT("cosm.ros.metrics.swarm_metrics_manager"),
      fs_output_manager(mconfig, output_root),
      mc_expected_counts({ { rmetrics::output_mode::ekAPPEND,
                             mconfig->csv.append.output_interval.v() },
                           { rmetrics::output_mode::ekTRUNCATE,
                             mconfig->csv.truncate.output_interval.v() },
                           { rmetrics::output_mode::ekCREATE,
                             mconfig->csv.create.output_interval.v() } }) {
  ER_INFO("cosm_msgs/* MD5: %s", cpal::kMsgTraitsMD5.c_str());

  /*
   * Register all standard metrics which don't require additional parameters,
   * and can by done by default.
   */
  register_standard(mconfig, n_robots);

  /*
   * Because each robot has their own topic for all metrics, we can wait for all
   * connections to become active before continuing (correctness by
   * construction).
   */
  ER_INFO("Waiting for all subscriber connections");
  std::all_of(std::begin(m_subs), std::end(m_subs), [&](const auto& sub) {
    return wait_for_connection(sub);
  });
  ER_INFO("All subscribers connected");
}

/*******************************************************************************
 * Initialization Functions
 ******************************************************************************/
void swarm_metrics_manager::register_standard(
    const rmconfig::metrics_config* mconfig,
    size_t n_robots) {
  ER_INFO("Register standard collectors: swarm_size=%zu", n_robots);

  using sink_list = rmpl::typelist<
      rmpl::identity<cfsm::metrics::block_transporter_metrics_csv_sink>,
      rmpl::identity<cforaging::metrics::block_transportee_metrics_csv_sink>,
    rmpl::identity<cspatial::metrics::interference_metrics_csv_sink>,
    rmpl::identity<chsensors::metrics::battery_metrics_csv_sink>
    >;

  rmetrics::register_with_sink<cros::metrics::swarm_metrics_manager,
                               rmetrics::file_sink_registerer>
      file(this, library().kStandard);
  rmetrics::register_using_config<decltype(file), rmconfig::file_sink_config>
      registerer(std::move(file), &mconfig->csv);

  boost::mpl::for_each<sink_list>(registerer);

  /* initialize counting map to track received metrics */
  m_tracking.init(cmspecs::spatial::kInterferenceCounts.scoped());
  m_tracking.init(cmspecs::blocks::kTransporter.scoped());
  m_tracking.init(cmspecs::blocks::kTransportee.scoped());
  m_tracking.init(cmspecs::sensors::kBattery.scoped());

  /* set ROS callbacks for metric collection */
  ::ros::NodeHandle n;
  auto cb = [&](cros::topic robot_ns) {
    m_subs.push_back(n.subscribe<crsmetrics::interference_metrics_msg>(
        robot_ns / cmspecs::spatial::kInterferenceCounts.scoped(),
        kQueueBufferSize,
        &swarm_metrics_manager::collect,
        this));
    m_subs.push_back(n.subscribe<crfsm::metrics::block_transporter_metrics_msg>(
        robot_ns / cmspecs::blocks::kTransporter.scoped(),
        kQueueBufferSize,
        &swarm_metrics_manager::collect,
        this));
    m_subs.push_back(n.subscribe<crfmetrics::block_transportee_metrics_msg>(
        robot_ns / cmspecs::blocks::kTransportee.scoped(),
        kQueueBufferSize,
        &swarm_metrics_manager::collect,
        this));
    m_subs.push_back(n.subscribe<chros::sensors::metrics::battery_metrics_msg>(
        robot_ns / cmspecs::sensors::kBattery.scoped(),
        kQueueBufferSize,
        &swarm_metrics_manager::collect,
        this));
  };
  cpros::swarm_iterator::robots(n_robots, cb);
  ER_INFO("Finished registering standard collectors");
} /* register_standard() */

void swarm_metrics_manager::register_with_n_robots(
    const rmconfig::metrics_config* mconfig,
    size_t n_robots) {
  ER_INFO("Register standard collectors: swarm_size=%zu", n_robots);

  using sink_typelist = rmpl::typelist<
    rmpl::identity<ckmetrics::kinematics_metrics_dist_csv_sink>,
    rmpl::identity<ckmetrics::kinematics_metrics_avg_csv_sink>
      >;

  auto extra_args = std::make_tuple(n_robots, ckmetrics::context_type::ekMAX);
  rmetrics::register_with_sink<cros::metrics::swarm_metrics_manager,
                               rmetrics::file_sink_registerer,
                               decltype(extra_args)>
      file(this, library().kWithNBlockClusters, extra_args);
  rmetrics::register_using_config<decltype(file), rmconfig::file_sink_config>
      registerer(std::move(file), &mconfig->csv);
  boost::mpl::for_each<sink_typelist>(registerer);

  /* initialize counting map to track received metrics */
  m_tracking.init(cmspecs::kinematics::kAvg.scoped());
  m_tracking.init(cmspecs::kinematics::kDist.scoped());

  /* set ROS callbacks for metric collection */
  ::ros::NodeHandle n;
  auto cb = [&](cros::topic robot_ns) {
    m_subs.push_back(n.subscribe<crsmetrics::kinematics_metrics_msg>(
        robot_ns / cmspecs::kinematics::kAvg.scoped(),
        kQueueBufferSize,
        &swarm_metrics_manager::collect,
        this));
    m_subs.push_back(n.subscribe<crsmetrics::kinematics_metrics_msg>(
        robot_ns / cmspecs::kinematics::kDist.scoped(),
        kQueueBufferSize,
        &swarm_metrics_manager::collect,
        this));
  };
  cpros::swarm_iterator::robots(n_robots, cb);
} /* register_with_n_robots() */

void swarm_metrics_manager::register_with_n_block_clusters(
    const rmconfig::metrics_config* mconfig,
    size_t n_robots,
    size_t n_clusters) {
  ER_INFO("Register block cluster collectors: swarm_size=%zu, n_clusters=%zu",
          n_robots,
          n_clusters);
  using sink_typelist = rmpl::typelist<
      rmpl::identity<cforaging::metrics::block_cluster_metrics_csv_sink> >;

  auto extra_args = std::make_tuple(n_clusters);
  rmetrics::register_with_sink<cros::metrics::swarm_metrics_manager,
                               rmetrics::file_sink_registerer,
                               decltype(extra_args)>
      file(this, library().kWithNBlockClusters, extra_args);
  rmetrics::register_using_config<decltype(file), rmconfig::file_sink_config>
      registerer(std::move(file), &mconfig->csv);
  boost::mpl::for_each<sink_typelist>(registerer);

  /* set ROS callbacks for metric collection */
  ::ros::NodeHandle n;
  auto cb = [&](cros::topic robot_ns) {
    ::ros::SubscribeOptions opts;
    using metrics_msg = crfmetrics::block_cluster_metrics_msg;

    /* initialize counting map to track received metrics */
    m_tracking.init(cmspecs::blocks::kClusters.scoped());

    auto factory = [&]() { return boost::make_shared<metrics_msg>(n_clusters); };
    auto collect_cb = std::bind(static_cast<void (swarm_metrics_manager::*)(
                                    const boost::shared_ptr<const metrics_msg>&)>(
                                    &swarm_metrics_manager::collect),
                                this,
                                std::placeholders::_1);
    opts.template init<metrics_msg>(robot_ns /
                                        cmspecs::blocks::kClusters.scoped(),
                                    kQueueBufferSize,
                                    collect_cb,
                                    factory);
    m_subs.push_back(n.subscribe(opts));
  };
  cpros::swarm_iterator::robots(n_robots, cb);
} /* register_with_n_block_clusters() */

bool swarm_metrics_manager::wait_for_connection(const ::ros::Subscriber& sub) {
  while (0 == sub.getNumPublishers()) {
    ER_ASSERT(::ros::ok(),
              "Unable to wait for subscriber connection--ros::ok() failed");

    /* For real robots, things take a while to come up so we have to wait */
    ::ros::spinOnce();
    ::ros::Duration(1.0).sleep();

    ER_DEBUG("Wait for topic '%s' subscription to activate",
             sub.getTopic().c_str());
  } /* for(&sub..) */

  return true;
} /* wait_for_connection() */

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool swarm_metrics_manager::flush(const rmetrics::output_mode& mode,
                                  const rtypes::timestep&) {
  for (auto& key : *collector_map() | boost::adaptors::map_keys) {
    auto* collector = get(key);
    if (collector->output_mode() != mode) {
      continue;
    }
    auto& tracking = m_tracking[key];
    if (!tracking.missing.empty()) {
      ER_WARN("Collector '%s' missing %zu packets",
              key.c_str(),
              tracking.missing.size());
    }
    if (tracking.n_received >= mc_expected_counts.at(mode)) {
      ER_DEBUG("Collector '%s' ready to flush: received=%zu,expecting=%zu",
               key.c_str(),
               tracking.n_received,
               mc_expected_counts.at(mode));
      if (tracking.n_received > mc_expected_counts.at(mode)) {
        ER_WARN("Collector '%s' received counts overflow: "
                "received=%zu,expecting=%zu",
                key.c_str(),
                tracking.n_received,
                mc_expected_counts.at(mode));
      }
      tracking.flush_ts = rtypes::timestep((tracking.interval_index + 1) *
                                           mc_expected_counts.at(mode));
      ER_DEBUG("Flushing collector '%s': interval_index=%zu, counts=%zu, ts=%zu",
               key.c_str(),
               tracking.interval_index,
               mc_expected_counts.at(mode),
               tracking.flush_ts.v());
      ER_ASSERT(collector->flush(tracking.flush_ts),
                "Collector '%s' did not flush when ready",
                key.c_str());
      tracking.flushed_collector = true;
    } else {
      ER_TRACE("Collector '%s' not ready to flush: received=%zu,expecting=%zu",
               key.c_str(),
               tracking.n_received,
               mc_expected_counts.at(mode));
    }
  } /* for(&key..) */
  return true;
} /* flush() */

void swarm_metrics_manager::interval_reset(const rtypes::timestep&) {
  for (auto& key : *collector_map() | boost::adaptors::map_keys) {
    auto& tracking = m_tracking[key];
    if (tracking.flushed_collector) {
      ER_DEBUG("Reseting collector '%s': interval_index=%zu",
               key.c_str(),
               tracking.interval_index);
      auto* collector = get(key);
      collector->interval_reset(tracking.flush_ts);
      m_tracking.reset(key);
    }

  } /* for(&key..) */
} /* interval_reset() */

/*******************************************************************************
 * ROS Callbacks
 ******************************************************************************/
void swarm_metrics_manager::collect(
    const boost::shared_ptr<const crfmetrics::block_transportee_metrics_msg>& msg) {
  auto* collector = get<cforaging::metrics::block_transportee_metrics_collector>(
      cmspecs::blocks::kTransportee.scoped());
  m_tracking.update_on_receive(cmspecs::blocks::kTransportee.scoped(),
                               msg->header.seq);
  ER_DEBUG("Received '%s' metrics, seq=%u",
           cmspecs::blocks::kTransportee.scoped().c_str(),
           msg->header.seq);
  collector->collect(msg->data);
} /* collect() */

void swarm_metrics_manager::collect(
    const boost::shared_ptr<const crfsm::metrics::block_transporter_metrics_msg>&
        msg) {
  auto* collector = get<cfsm::metrics::block_transporter_metrics_collector>(
      cmspecs::blocks::kTransporter.scoped());
  m_tracking.update_on_receive(cmspecs::blocks::kTransporter.scoped(),
                               msg->header.seq);
  ER_DEBUG("Received '%s' metrics, seq=%u",
           cmspecs::blocks::kTransporter.scoped().c_str(),
           msg->header.seq);
  collector->collect(msg->data);
} /* collect() */

void swarm_metrics_manager::collect(
    const boost::shared_ptr<const crfmetrics::block_cluster_metrics_msg>& msg) {
  auto* collector = get<cforaging::metrics::block_cluster_metrics_collector>(
      cmspecs::blocks::kClusters.scoped());
  m_tracking.update_on_receive(cmspecs::blocks::kClusters.scoped(),
                               msg->header.seq);
  ER_DEBUG("Received '%s' metrics, seq=%u",
           cmspecs::blocks::kClusters.scoped().c_str(),
           msg->header.seq);
  collector->collect(msg->data);
} /* collect() */

void swarm_metrics_manager::collect(
    const boost::shared_ptr<const crsmetrics::kinematics_metrics_msg>& msg) {
  m_tracking.update_on_receive(cmspecs::kinematics::kAvg.scoped(),
                               msg->header.seq);
  m_tracking.update_on_receive(cmspecs::kinematics::kDist.scoped(),
                               msg->header.seq);
  ER_DEBUG("Received '%s' metrics, seq=%u",
           cmspecs::kinematics::kAvg.scoped().c_str(),
           msg->header.seq);
  ER_DEBUG("Received '%s' metrics, seq=%u",
           cmspecs::kinematics::kDist.scoped().c_str(),
           msg->header.seq);

  auto* collector = get<ckin::metrics::kinematics_metrics_collector>(
      cmspecs::kinematics::kAvg.scoped());
  collector->collect(msg->data);

  collector = get<ckin::metrics::kinematics_metrics_collector>(
      cmspecs::kinematics::kDist.scoped());
  collector->collect(msg->data);
} /* collect() */
void swarm_metrics_manager::collect(
    const boost::shared_ptr<const crsmetrics::interference_metrics_msg>& msg) {
  auto* collector = get<csmetrics::interference_metrics_collector>(
      cmspecs::spatial::kInterferenceCounts.scoped());
  m_tracking.update_on_receive(cmspecs::spatial::kInterferenceCounts.scoped(),
                               msg->header.seq);
  ER_DEBUG("Received '%s' metrics, seq=%u",
           cmspecs::spatial::kInterferenceCounts.scoped().c_str(),
           msg->header.seq);
  collector->collect(msg->data);
} /* collect() */

void swarm_metrics_manager::collect(
    const boost::shared_ptr<const chros::sensors::metrics::battery_metrics_msg>& msg) {
  auto* collector = get<chsensors::metrics::battery_metrics_collector>(
      cmspecs::sensors::kBattery.scoped());
  m_tracking.update_on_receive(cmspecs::sensors::kBattery.scoped(),
                               msg->header.seq);
  ER_DEBUG("Received '%s' metrics, seq=%u",
           cmspecs::sensors::kBattery.scoped().c_str(),
           msg->header.seq);
  collector->collect(msg->data);
} /* collect() */

} /* namespace cosm::ros::metrics */
