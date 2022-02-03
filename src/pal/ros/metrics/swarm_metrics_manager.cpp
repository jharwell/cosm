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
#include "cosm/controller/base_controllerQ3D.hpp"
#include "cosm/metrics/specs.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/ros/metrics/registrable.hpp"

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

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, ros, metrics);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
swarm_metrics_manager::swarm_metrics_manager(
    const rmconfig::metrics_config* const mconfig,
    const fs::path& output_root)
    : ER_CLIENT_INIT("cosm.ros.metrics.swarm_metrics_manager"),
      fs_output_manager(mconfig, output_root) {
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
void swarm_metrics_manager::collect_from_block(cros::topic robot_ns) {
  /* collect(cmspecs::blocks::kTransportee.scoped, *block->md()); */
} /* collect_from_block() */

void swarm_metrics_manager::collect_from_controller(cros::topic robot_ns) {
  /* collect(cmspecs::spatial::kMovement.scoped, *controller); */
  /* collect(cmspecs::spatial::kInterferenceCounts.scoped, */
  /*         *controller->inta_tracker()); */
} /* collect_from_controller() */

void swarm_metrics_manager::register_standard(
    const rmconfig::metrics_config* mconfig) {
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
} /* register_standard() */

void swarm_metrics_manager::register_with_n_block_clusters(
    const rmconfig::metrics_config* mconfig,
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
} /* register_with_n_block_clusters() */

NS_END(metrics, ros, cosm);
