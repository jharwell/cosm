/**
 * \file swarm_metrics_manager.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ros/metrics/swarm_metrics_manager.hpp"

#include <boost/mpl/for_each.hpp>

#include "rcppsw/metrics/register_with_sink.hpp"
#include "rcppsw/metrics/register_using_config.hpp"
#include "rcppsw/metrics/file_sink_registerer.hpp"
#include "rcppsw/mpl/identity.hpp"
#include "rcppsw/mpl/typelist.hpp"

#include "cosm/controller/base_controller2D.hpp"
#include "cosm/metrics/specs.hpp"
#include "cosm/ros/metrics/registrable.hpp"
#include "cosm/pal/ros/swarm_iterator.hpp"

#include "cosm/foraging/metrics/block_cluster_metrics_csv_sink.hpp"
#include "cosm/fsm/metrics/block_transporter_metrics_csv_sink.hpp"
#include "cosm/foraging/metrics/block_transportee_metrics_csv_sink.hpp"
#include "cosm/spatial/metrics/interference_metrics_csv_sink.hpp"
#include "cosm/spatial/metrics/movement_metrics_csv_sink.hpp"
#include "cosm/foraging/metrics/block_cluster_metrics_collector.hpp"
#include "cosm/fsm/metrics/block_transporter_metrics_collector.hpp"
#include "cosm/foraging/metrics/block_transportee_metrics_collector.hpp"
#include "cosm/spatial/metrics/interference_metrics_collector.hpp"
#include "cosm/spatial/metrics/movement_metrics.hpp"
#include "cosm/spatial/metrics/movement_metrics_collector.hpp"
#include "cosm/ros/spatial/metrics/movement_metrics_glue.hpp"
#include "cosm/ros/spatial/metrics/interference_metrics_glue.hpp"
#include "cosm/ros/fsm/metrics/block_transporter_metrics_glue.hpp"
#include "cosm/ros/foraging/metrics/block_transportee_metrics_glue.hpp"
#include "cosm/ros/foraging/metrics/block_cluster_metrics_glue.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, ros, metrics);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
swarm_metrics_manager::swarm_metrics_manager(
    const rmconfig::metrics_config* const mconfig,
    const fs::path& output_root,
    size_t n_robots)
    : ER_CLIENT_INIT("cosm.ros.metrics.swarm_metrics_manager"),
      fs_output_manager(mconfig, output_root) {
  /*
   * Register all standard metrics which don't require additional parameters,
   * and can by done by default.
   */
  register_standard(mconfig, n_robots);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void swarm_metrics_manager::register_standard(
    const rmconfig::metrics_config* mconfig,
    size_t n_robots) {
  using sink_list = rmpl::typelist<
    rmpl::identity<cspatial::metrics::movement_metrics_csv_sink>,
    rmpl::identity<cfsm::metrics::block_transporter_metrics_csv_sink>,
    rmpl::identity<cforaging::metrics::block_transportee_metrics_csv_sink>,
    rmpl::identity<cspatial::metrics::interference_metrics_csv_sink>
    >;


  rmetrics::register_with_sink<cros::metrics::swarm_metrics_manager,
                               rmetrics::file_sink_registerer> file(this,
                                                                    registrable::kStandard);
  rmetrics::register_using_config<decltype(file),
                                  rmconfig::file_sink_config> registerer(
                                      std::move(file),
                                      &mconfig->csv);

  boost::mpl::for_each<sink_list>(registerer);

  /* set ROS callbacks for metric collection */
  ::ros::NodeHandle n;
  auto cb = [&](cros::topic robot_ns) {
              m_subs.push_back(n.subscribe<cspatial::metrics::movement_metrics_data>(
                  robot_ns / cmspecs::spatial::kMovement.scoped,
                  kQueueBufferSize,
                  &swarm_metrics_manager::collect,
                  this));
              m_subs.push_back(n.subscribe<cspatial::metrics::interference_metrics_data>(
                  robot_ns / cmspecs::spatial::kInterferenceCounts.scoped,
                  kQueueBufferSize,
                  &swarm_metrics_manager::collect,
                  this));
              m_subs.push_back(n.subscribe<cfsm::metrics::block_transporter_metrics_data>(
                  robot_ns / cmspecs::blocks::kTransporter.scoped,
                  kQueueBufferSize,
                  &swarm_metrics_manager::collect,
                  this));
              m_subs.push_back(n.subscribe<cforaging::metrics::block_transportee_metrics_data>(
                  robot_ns / cmspecs::blocks::kTransportee.scoped,
                  kQueueBufferSize,
                  &swarm_metrics_manager::collect,
                  this));
            };
  cpros::swarm_iterator::robots(n_robots, cb);
} /* register_standard() */

void swarm_metrics_manager::register_with_n_block_clusters(
    const rmconfig::metrics_config* mconfig,
    size_t n_robots,
    size_t n_clusters) {
  using sink_typelist = rmpl::typelist<
    rmpl::identity<cforaging::metrics::block_cluster_metrics_csv_sink>
    >;

    auto extra_args = std::make_tuple(n_clusters);
  rmetrics::register_with_sink<cros::metrics::swarm_metrics_manager,
                               rmetrics::file_sink_registerer,
                               decltype(extra_args)> file(this,
                                                         registrable::kWithNBlockClusters,
                                                         extra_args);
  rmetrics::register_using_config<decltype(file),
                                  rmconfig::file_sink_config> registerer(
                                      std::move(file),
                                      &mconfig->csv);
  boost::mpl::for_each<sink_typelist>(registerer);

  /* set ROS callbacks for metric collection */
  ::ros::NodeHandle n;
  auto cb = [&](cros::topic robot_ns) {
  ::ros::SubscribeOptions opts;
  using metrics_data = cforaging::metrics::block_cluster_metrics_data;

  auto factory = [&](){ return boost::make_shared<metrics_data>(n_clusters); };
  auto collect_cb = std::bind(static_cast<void(swarm_metrics_manager::*)(const boost::shared_ptr<const metrics_data>&)>(&swarm_metrics_manager::collect),
                                this,
                                std::placeholders::_1);
  opts.template init<metrics_data>(robot_ns / cmspecs::blocks::kClusters.scoped,
                                   kQueueBufferSize,
                                   collect_cb,
                                   factory);
  m_subs.push_back(n.subscribe(opts));
            };
  cpros::swarm_iterator::robots(n_robots, cb);
} /* register_with_n_block_clusters() */

/*******************************************************************************
 * ROS Callbacks
 ******************************************************************************/
void swarm_metrics_manager::collect(
    const boost::shared_ptr<const cforaging::metrics::block_transportee_metrics_data>& in) {
  auto* collector = get<cforaging::metrics::block_transportee_metrics_collector>(
      cmspecs::blocks::kTransportee.scoped);
  collector->data(*in);
} /* collect() */

void swarm_metrics_manager::collect(
    const boost::shared_ptr<const cfsm::metrics::block_transporter_metrics_data>& in) {
  auto* collector = get<cfsm::metrics::block_transporter_metrics_collector>(
      cmspecs::blocks::kTransporter.scoped);
  collector->data(*in);
} /* collect() */

void swarm_metrics_manager::collect(
    const boost::shared_ptr<const cforaging::metrics::block_cluster_metrics_data>& in) {
  auto* collector = get<cforaging::metrics::block_cluster_metrics_collector>(
      cmspecs::blocks::kClusters.scoped);
  collector->data(*in);
} /* collect() */

void swarm_metrics_manager::collect(
    const boost::shared_ptr<const csmetrics::movement_metrics_data>& in) {
  auto* collector = get<csmetrics::movement_metrics_collector>(
      cmspecs::spatial::kMovement.scoped);
  collector->data(*in);
} /* collect() */
void swarm_metrics_manager::collect(
    const boost::shared_ptr<const csmetrics::interference_metrics_data>& in) {
  auto* collector = get<csmetrics::interference_metrics_collector>(
      cmspecs::spatial::kMovement.scoped);
  collector->data(*in);
} /* collect() */



NS_END(metrics, ros, cosm);
