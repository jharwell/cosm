/**
 * \file cosm_metrics_manager.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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
#include "cosm/metrics/cosm_metrics_manager.hpp"

#include <boost/mpl/for_each.hpp>

#include "rcppsw/metrics/collector_registerer.hpp"

#include "cosm/arena/base_arena_map.hpp"
#include "cosm/controller/base_controller2D.hpp"
#include "cosm/controller/base_controllerQ3D.hpp"
#include "cosm/convergence/convergence_calculator.hpp"
#include "cosm/foraging/repr/block_cluster.hpp"
#include "cosm/foraging/block_motion_handler.hpp"
#include "cosm/repr/base_block3D.hpp"

#include "cosm/convergence/metrics/convergence_metrics.hpp"
#include "cosm/convergence/metrics/convergence_metrics_collector.hpp"
#include "cosm/convergence/metrics/convergence_metrics_csv_sink.hpp"
#include "cosm/foraging/block_dist/base_distributor.hpp"
#include "cosm/foraging/block_dist/metrics/distributor_metrics.hpp"
#include "cosm/foraging/block_dist/metrics/distributor_metrics_collector.hpp"
#include "cosm/foraging/block_dist/metrics/distributor_metrics_csv_sink.hpp"
#include "cosm/foraging/metrics/block_cluster_metrics_csv_sink.hpp"
#include "cosm/foraging/metrics/block_cluster_metrics_collector.hpp"
#include "cosm/foraging/metrics/block_motion_metrics_collector.hpp"
#include "cosm/foraging/metrics/block_motion_metrics_csv_sink.hpp"
#include "cosm/fsm/metrics/block_transporter_metrics_collector.hpp"
#include "cosm/fsm/metrics/block_transporter_metrics_csv_sink.hpp"
#include "cosm/foraging/metrics/block_transportee_metrics_collector.hpp"
#include "cosm/foraging/metrics/block_transportee_metrics_csv_sink.hpp"
#include "cosm/spatial/metrics/dist2D_pos_metrics_collector.hpp"
#include "cosm/spatial/metrics/dist2D_pos_metrics_csv_sink.hpp"
#include "cosm/spatial/metrics/dist3D_metrics.hpp"
#include "cosm/spatial/metrics/dist3D_pos_metrics_collector.hpp"
#include "cosm/spatial/metrics/dist3D_pos_metrics_csv_sink.hpp"
#include "cosm/spatial/metrics/explore_locs2D_metrics_collector.hpp"
#include "cosm/spatial/metrics/explore_locs2D_metrics_csv_sink.hpp"
#include "cosm/spatial/metrics/explore_locs3D_metrics_collector.hpp"
#include "cosm/spatial/metrics/explore_locs3D_metrics_csv_sink.hpp"
#include "cosm/spatial/metrics/goal_acq_locs2D_metrics_collector.hpp"
#include "cosm/spatial/metrics/goal_acq_locs2D_metrics_csv_sink.hpp"
#include "cosm/spatial/metrics/goal_acq_metrics_collector.hpp"
#include "cosm/spatial/metrics/goal_acq_metrics_csv_sink.hpp"
#include "cosm/spatial/metrics/interference_locs2D_metrics_collector.hpp"
#include "cosm/spatial/metrics/interference_locs2D_metrics_csv_sink.hpp"
#include "cosm/spatial/metrics/interference_locs3D_metrics_collector.hpp"
#include "cosm/spatial/metrics/interference_locs3D_metrics_csv_sink.hpp"
#include "cosm/spatial/metrics/interference_metrics_collector.hpp"
#include "cosm/spatial/metrics/interference_metrics_csv_sink.hpp"
#include "cosm/spatial/metrics/movement_metrics.hpp"
#include "cosm/spatial/metrics/movement_metrics_collector.hpp"
#include "cosm/spatial/metrics/movement_metrics_csv_sink.hpp"
#include "cosm/spatial/metrics/vector_locs2D_metrics_collector.hpp"
#include "cosm/spatial/metrics/vector_locs2D_metrics_csv_sink.hpp"
#include "cosm/spatial/metrics/vector_locs3D_metrics_collector.hpp"
#include "cosm/spatial/metrics/vector_locs3D_metrics_csv_sink.hpp"
#include "cosm/spatial/strategy/metrics/nest_acq_metrics_collector.hpp"
#include "cosm/spatial/strategy/metrics/nest_acq_metrics_csv_sink.hpp"
#include "cosm/tv/metrics/population_dynamics_metrics_collector.hpp"
#include "cosm/tv/metrics/population_dynamics_metrics_csv_sink.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, metrics);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
cosm_metrics_manager::cosm_metrics_manager(
    const rmconfig::metrics_config* const mconfig,
    const fs::path& output_root)
    : base_metrics_manager(mconfig, output_root),
      ER_CLIENT_INIT("cosm.metrics.cosm_metrics_manager") {
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
void cosm_metrics_manager::collect_from_block(
    const crepr::base_block3D* const block) {
  collect("blocks::transportee", *block->md());
} /* collect_from_block() */

void cosm_metrics_manager::collect_from_controller(
    const controller::base_controller2D* const controller) {
  collect("swarm::spatial_dist2D::pos", *controller);
} /* collect_from_controller() */

void cosm_metrics_manager::collect_from_controller(
    const controller::base_controllerQ3D* const controller) {
  collect("swarm::spatial_dist3D::pos", *controller);
} /* collect_from_controller() */

void cosm_metrics_manager::collect_from_arena(
    const carena::base_arena_map* const map) {
  collect("blocks::motion", *map->block_motion_handler());
  collect("blocks::distributor", *map->block_distributor());
  for (auto* cluster : map->block_distributor()->block_clustersro()) {
    collect("blocks::clusters", *cluster);
  } /* for(&cluster..) */
} /* collect_from_arena() */

void cosm_metrics_manager::register_standard(
    const rmconfig::metrics_config* mconfig) {
  using sink_list = rmpl::typelist<
    rmpl::identity<cconvergence::metrics::convergence_metrics_csv_sink>,
    rmpl::identity<csmetrics::movement_metrics_csv_sink>,
    rmpl::identity<cfbd::metrics::distributor_metrics_csv_sink>,
    rmpl::identity<cfmetrics::block_motion_metrics_csv_sink>,
    rmpl::identity<cfsm::metrics::block_transporter_metrics_csv_sink>,
    rmpl::identity<cfmetrics::block_transportee_metrics_csv_sink>,
    rmpl::identity<csmetrics::goal_acq_metrics_csv_sink>,
    rmpl::identity<csmetrics::interference_metrics_csv_sink>,
    rmpl::identity<cssmetrics::nest_acq_metrics_csv_sink>,
    rmpl::identity<ctvmetrics::population_dynamics_metrics_csv_sink>
    >;

  rmetrics::creatable_collector_set creatable_set = {
    { typeid(cconvergence::metrics::convergence_metrics_collector),
      "swarm_convergence",
      "swarm::convergence",
      rmetrics::output_mode::ekAPPEND },
    { typeid(csmetrics::movement_metrics_collector),
      "spatial_movement",
      "spatial::movement",
      rmetrics::output_mode::ekAPPEND },
    { typeid(cfbd::metrics::distributor_metrics_collector),
      "block_distributor",
      "blocks::distributor",
      rmetrics::output_mode::ekAPPEND },
    { typeid(cfmetrics::block_motion_metrics_collector),
      "block_motion",
      "blocks::motion",
      rmetrics::output_mode::ekAPPEND },
    { typeid(cfsm::metrics::block_transporter_metrics_collector),
      "block_transporter",
      "blocks::transporter",
      rmetrics::output_mode::ekAPPEND },
    { typeid(cfmetrics::block_transportee_metrics_collector),
      "block_transportee",
      "blocks::transportee",
      rmetrics::output_mode::ekAPPEND },
    { typeid(csmetrics::goal_acq_metrics_collector),
      "block_acq_counts",
      "blocks::acq_counts",
      rmetrics::output_mode::ekAPPEND },
    { typeid(csmetrics::interference_metrics_collector),
      "fsm_interference_counts",
      "fsm::interference_counts",
      rmetrics::output_mode::ekAPPEND },

    {typeid(cssmetrics::nest_acq_metrics_collector),
    "nest_acq_strategy",
    "strategy::nest_acq",
    rmetrics::output_mode::ekAPPEND },
    { typeid(ctvmetrics::population_dynamics_metrics_collector),
      "tv_population",
      "tv::population",
      rmetrics::output_mode::ekAPPEND }
  };

  rmetrics::register_with_csv_sink csv(&mconfig->csv,
                             creatable_set,
                             this);
  rmetrics::collector_registerer<decltype(csv)> registerer(std::move(csv));
  boost::mpl::for_each<sink_list>(registerer);
} /* register_standard() */

void cosm_metrics_manager::register_with_arena_dims2D(
    const rmconfig::metrics_config* mconfig,
    const rmath::vector2z& dims) {
  using sink_typelist = rmpl::typelist<
    rmpl::identity<csmetrics::dist2D_pos_metrics_csv_sink>,
    rmpl::identity<csmetrics::explore_locs2D_metrics_csv_sink>,
    rmpl::identity<csmetrics::goal_acq_locs2D_metrics_csv_sink>,
    rmpl::identity<csmetrics::interference_locs2D_metrics_csv_sink>,
    rmpl::identity<csmetrics::vector_locs2D_metrics_csv_sink>
    >;

  rmetrics::creatable_collector_set creatable_set = {
    { typeid(csmetrics::dist2D_pos_metrics_collector),
      "swarm_dist2D_pos",
      "swarm::spatial_dist2D::pos",
      rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE },
    { typeid(csmetrics::explore_locs2D_metrics_collector),
      "block_acq_explore_locs2D",
      "blocks::acq_explore_locs2D",
      rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE },
    { typeid(csmetrics::goal_acq_locs2D_metrics_collector),
      "block_acq_locs2D",
      "blocks::acq_locs2D",
      rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE },
    { typeid(csmetrics::interference_locs2D_metrics_collector),
      "fsm_interference_locs2D",
      "fsm::interference_locs2D",
      rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE },

    { typeid(csmetrics::vector_locs2D_metrics_collector),
      "block_acq_vector_locs2D",
      "blocks::acq_vector_locs2D",
      rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE },
  };

  auto extra_args = std::make_tuple(dims);
  rmetrics::register_with_csv_sink<decltype(extra_args)> csv(&mconfig->csv,
                                                             creatable_set,
                                                             this,
                                                             extra_args);

  rmetrics::collector_registerer<decltype(csv)> registerer(std::move(csv));
  boost::mpl::for_each<sink_typelist>(registerer);
} /* register_with_arena_dims2D() */

void cosm_metrics_manager::register_with_arena_dims3D(
    const rmconfig::metrics_config* mconfig,
    const rmath::vector3z& dims) {
  using sink_typelist = rmpl::typelist<
    rmpl::identity<csmetrics::dist3D_pos_metrics_csv_sink>,
    rmpl::identity<csmetrics::explore_locs3D_metrics_csv_sink>,
    rmpl::identity<csmetrics::interference_locs3D_metrics_csv_sink>,
    rmpl::identity<csmetrics::vector_locs3D_metrics_csv_sink>
      >;

  rmetrics::creatable_collector_set creatable_set = {
    { typeid(csmetrics::dist3D_pos_metrics_collector),
      "swarm_dist3D_pos",
      "swarm::spatial_dist3D::pos",
      rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE },
    { typeid(csmetrics::explore_locs3D_metrics_collector),
      "block_acq_explore_locs3D",
      "blocks::acq_explore_locs3D",
      rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE },
    { typeid(csmetrics::interference_locs3D_metrics_collector),
      "fsm_interference_locs3D",
      "fsm::interference_locs3D",
      rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE },
    { typeid(csmetrics::vector_locs3D_metrics_collector),
      "block_acq_vector_locs3D",
      "blocks::acq_vector_locs3D",
      rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE }
  };

  auto extra_args = std::make_tuple(dims);
  rmetrics::register_with_csv_sink<decltype(extra_args)> csv(&mconfig->csv,
                                                             creatable_set,
                                                             this,
                                                             extra_args);

  rmetrics::collector_registerer<decltype(csv)> registerer(std::move(csv));
  boost::mpl::for_each<sink_typelist>(registerer);
} /* register_with_arena_dims3D() */

void cosm_metrics_manager::register_with_n_block_clusters(
    const rmconfig::metrics_config* mconfig,
    size_t n_clusters) {
  using sink_typelist = rmpl::typelist<
    rmpl::identity<cfmetrics::block_cluster_metrics_csv_sink>
    >;

  rmetrics::creatable_collector_set creatable_set = {
    { typeid(cfmetrics::block_cluster_metrics_collector),
      "block_clusters",
      "blocks::clusters",
      rmetrics::output_mode::ekAPPEND }
  };
  auto extra_args = std::make_tuple(n_clusters);
  rmetrics::register_with_csv_sink<decltype(extra_args)> csv(&mconfig->csv,
                                                             creatable_set,
                                                             this,
                                                             extra_args);

  rmetrics::collector_registerer<decltype(csv)> registerer(std::move(csv));
  boost::mpl::for_each<sink_typelist>(registerer);
} /* register_with_n_block_clusters() */

NS_END(metrics, cosm);
