/**
 * \file robot_metrics_manager.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ros/metrics/robot_metrics_manager.hpp"

#include <boost/mpl/for_each.hpp>

#include "rcppsw/metrics/network_sink_registerer.hpp"
#include "rcppsw/metrics/register_using_config.hpp"
#include "rcppsw/metrics/register_with_sink.hpp"
#include "rcppsw/mpl/identity.hpp"
#include "rcppsw/mpl/typelist.hpp"

#include "cosm/controller/base_controller2D.hpp"
#include "cosm/controller/base_controllerQ3D.hpp"
#include "cosm/foraging/metrics/block_cluster_metrics_collector.hpp"
#include "cosm/foraging/metrics/block_transportee_metrics_collector.hpp"
#include "cosm/fsm/metrics/block_transporter_metrics_collector.hpp"
#include "cosm/metrics/specs.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/ros/foraging/metrics/block_cluster_metrics_topic_sink.hpp"
#include "cosm/ros/foraging/metrics/block_transportee_metrics_topic_sink.hpp"
#include "cosm/ros/fsm/metrics/block_transporter_metrics_topic_sink.hpp"
#include "cosm/ros/metrics/registrable.hpp"
#include "cosm/ros/spatial/metrics/interference_metrics_topic_sink.hpp"
#include "cosm/ros/kin/metrics/kinematics_metrics_topic_sink.hpp"
#include "cosm/spatial/metrics/interference_metrics_collector.hpp"
#include "cosm/kin/metrics/kinematics_metrics.hpp"
#include "cosm/kin/metrics/kinematics_metrics_collector.hpp"
#include "cosm/subsystem/saa_subsystemQ3D.hpp"
#include "cosm/subsystem/sensing_subsystem.hpp"
#include "cosm/hal/sensors/metrics/battery_metrics_collector.hpp"
#include "cosm/hal/ros/sensors/metrics/battery_metrics_topic_sink.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::ros::metrics {

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
robot_metrics_manager::robot_metrics_manager(
    const cros::topic& robot_ns,
    const rmconfig::metrics_config* const mconfig)
    : ER_CLIENT_INIT("cosm.ros.metrics.robot_metrics_manager"),
      network_output_manager(robot_ns.string() + "/") {
  ER_INFO("cosm_msgs/* MD5: %s", cpal::kMsgTraitsMD5.c_str());
  /*
   * Register all standard metrics which don't require additional parameters,
   * and can by done by default.
   */
  register_standard(mconfig);

  /* setup metric collection for all collector groups in all sink groups */
  initialize();
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void robot_metrics_manager::collect_from_block(
    const crepr::base_block3D* const block) {
  collect(cmspecs::blocks::kTransportee.scoped(), *block->md());
} /* collect_from_block() */

void robot_metrics_manager::collect_from_controller(
    const ccontroller::base_controller2D* const controller) {
  ER_DEBUG("Collect metrics from robot%d", controller->entity_id().v());
  collect(cmspecs::kinematics::kAvg.scoped(), *controller);
  collect(cmspecs::kinematics::kDist.scoped(), *controller);
  collect(cmspecs::spatial::kInterferenceCounts.scoped(),
          *controller->inta_tracker());
  auto battery = controller->saa()->sensing()->battery();
  if (nullptr != battery) {
    collect(cmspecs::sensors::kBattery.scoped(), *battery);
  }
} /* collect_from_controller() */

void robot_metrics_manager::register_standard(
    const rmconfig::metrics_config* mconfig) {
  using sink_list = rmpl::typelist<
    rmpl::identity<chros::sensors::metrics::battery_metrics_topic_sink>,
      rmpl::identity<cros::fsm::metrics::block_transporter_metrics_topic_sink>,
      rmpl::identity<cros::foraging::metrics::block_transportee_metrics_topic_sink>,
      rmpl::identity<cros::spatial::metrics::interference_metrics_topic_sink> >;
  ER_DEBUG("Register standard metric collectors");

  rmetrics::register_with_sink<cros::metrics::robot_metrics_manager,
                               rmetrics::network_sink_registerer>
      net(this, registrable::kStandard);
  rmetrics::register_using_config<decltype(net), rmconfig::network_sink_config>
      registerer(std::move(net), &mconfig->network);

  boost::mpl::for_each<sink_list>(registerer);
} /* register_standard() */

void robot_metrics_manager::register_with_n_robots(
    const rmconfig::metrics_config* mconfig,
    size_t n_robots) {
  ER_DEBUG("Register metric collectors with n_robots=%zu", n_robots);

  using sink_list = rmpl::typelist<
    rmpl::identity<cros::kin::metrics::kinematics_metrics_topic_sink>
     >;
  auto extra_args = std::make_tuple(n_robots, ckmetrics::context_type::ekMAX);

  rmetrics::register_with_sink<cros::metrics::robot_metrics_manager,
                               rmetrics::network_sink_registerer,
                               decltype(extra_args)>
      net(this, registrable::kWithNRobots, extra_args);
  rmetrics::register_using_config<decltype(net), rmconfig::network_sink_config>
      registerer(std::move(net), &mconfig->network);

  boost::mpl::for_each<sink_list>(registerer);
} /* register_standard() */

void robot_metrics_manager::register_with_n_block_clusters(
    const rmconfig::metrics_config* mconfig,
    size_t n_clusters) {
  ER_DEBUG("Register metric collectors with block clusters: n_clusters=%zu",
           n_clusters);
  using sink_typelist = rmpl::typelist<
      rmpl::identity<cros::foraging::metrics::block_cluster_metrics_topic_sink> >;

  auto extra_args = std::make_tuple(n_clusters);
  rmetrics::register_with_sink<cros::metrics::robot_metrics_manager,
                               rmetrics::network_sink_registerer,
                               decltype(extra_args)>
      net(this, registrable::kWithNBlockClusters, extra_args);
  rmetrics::register_using_config<decltype(net), rmconfig::network_sink_config>
      registerer(std::move(net), &mconfig->network);
  boost::mpl::for_each<sink_typelist>(registerer);
} /* register_with_n_block_clusters() */

} /* namespace cosm::ros::metrics */
