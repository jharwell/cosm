/**
 * \file fs_output_manager.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/argos/metrics/fs_output_manager.hpp"

#include <boost/mpl/for_each.hpp>

#include "rcppsw/metrics/file_sink_registerer.hpp"
#include "rcppsw/metrics/register_using_config.hpp"
#include "rcppsw/metrics/register_with_sink.hpp"
#include "rcppsw/mpl/identity.hpp"

#include "cosm/arena/base_arena_map.hpp"
#include "cosm/controller/base_controller2D.hpp"
#include "cosm/controller/base_controllerQ3D.hpp"
#include "cosm/controller/base_controller3D.hpp"
#include "cosm/convergence/convergence_calculator.hpp"
#include "cosm/convergence/metrics/convergence_metrics.hpp"
#include "cosm/convergence/metrics/convergence_metrics_collector.hpp"
#include "cosm/convergence/metrics/convergence_metrics_csv_sink.hpp"
#include "cosm/foraging/block_dist/base_distributor.hpp"
#include "cosm/foraging/block_dist/metrics/distributor_metrics.hpp"
#include "cosm/foraging/block_dist/metrics/distributor_metrics_collector.hpp"
#include "cosm/foraging/block_dist/metrics/distributor_metrics_csv_sink.hpp"
#include "cosm/foraging/block_motion_handler.hpp"
#include "cosm/foraging/metrics/block_cluster_metrics_collector.hpp"
#include "cosm/foraging/metrics/block_cluster_metrics_csv_sink.hpp"
#include "cosm/foraging/metrics/block_motion_metrics_collector.hpp"
#include "cosm/foraging/metrics/block_motion_metrics_csv_sink.hpp"
#include "cosm/foraging/metrics/block_transportee_metrics_collector.hpp"
#include "cosm/foraging/metrics/block_transportee_metrics_csv_sink.hpp"
#include "cosm/foraging/repr/block_cluster.hpp"
#include "cosm/fsm/metrics/block_transporter_metrics_collector.hpp"
#include "cosm/fsm/metrics/block_transporter_metrics_csv_sink.hpp"
#include "cosm/metrics/specs.hpp"
#include "cosm/repr/sim_block3D.hpp"
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
#include "cosm/kin/metrics/kinematics_metrics.hpp"
#include "cosm/kin/metrics_proxy.hpp"
#include "cosm/kin/metrics/kinematics_metrics_collector.hpp"
#include "cosm/kin/metrics/kinematics_metrics_avg_csv_sink.hpp"
#include "cosm/kin/metrics/kinematics_metrics_dist_csv_sink.hpp"
#include "cosm/kin/metrics/contexts.hpp"
#include "cosm/spatial/metrics/nest_zone_metrics_collector.hpp"
#include "cosm/spatial/metrics/nest_zone_metrics_csv_sink.hpp"
#include "cosm/spatial/metrics/vector_locs2D_metrics_collector.hpp"
#include "cosm/spatial/metrics/vector_locs2D_metrics_csv_sink.hpp"
#include "cosm/spatial/metrics/vector_locs3D_metrics_collector.hpp"
#include "cosm/spatial/metrics/vector_locs3D_metrics_csv_sink.hpp"
#include "cosm/spatial/strategy/nest/metrics/acq_metrics_collector.hpp"
#include "cosm/spatial/strategy/nest/metrics/acq_metrics_csv_sink.hpp"
#include "cosm/tv/metrics/population_dynamics_metrics_collector.hpp"
#include "cosm/tv/metrics/population_dynamics_metrics_csv_sink.hpp"
#include "cosm/subsystem/saa_subsystemQ3D.hpp"
#include "cosm/subsystem/sensing_subsystem.hpp"
#include "cosm/hal/sensors/metrics/battery_metrics_collector.hpp"
#include "cosm/hal/sensors/metrics/battery_metrics_csv_sink.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::argos::metrics {

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
fs_output_manager::fs_output_manager(const rmconfig::metrics_config* const mconfig,
                                     const fs::path& output_root)
    : rmetrics::fs_output_manager(mconfig, output_root),
      ER_CLIENT_INIT("cosm.argos.metrics.fs_output_manager") {
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
void fs_output_manager::collect_from_block(const crepr::sim_block3D* const block) {
  collect(cmspecs::blocks::kTransportee.scoped(), *block->md());
} /* collect_from_block() */

void fs_output_manager::collect_from_controller(
    const ccontroller::base_controller2D* const controller) {
  collect(cmspecs::spatial::kDistPosition2D.scoped(), *controller);
  collect(cmspecs::kinematics::kAvg.scoped(), *controller->kin_proxy());
  collect(cmspecs::kinematics::kDist.scoped(), *controller->kin_proxy());
  collect(cmspecs::spatial::kInterferenceCounts.scoped(),
          *controller->inta_tracker());
  collect_if(cmspecs::spatial::kInterferenceLocs2D.scoped(),
             *controller->inta_tracker(),
             [&](const rmetrics::base_metrics&) {
               return controller->inta_tracker()->exp_interference();
             });

#if defined(COSM_HAL_TARGET_HAS_BATTERY_SENSOR)
  auto battery = controller->saa()->sensing()->battery();
  if (nullptr != battery) {
    collect(cmspecs::sensors::kBattery.scoped(), *battery);
  }
#endif
} /* collect_from_controller() */

void fs_output_manager::collect_from_controller(
    const ccontroller::base_controllerQ3D* const controller) {
  collect(cmspecs::spatial::kDistPosition3D.scoped(), *controller);
  collect(cmspecs::kinematics::kAvg.scoped(), *controller->kin_proxy());
  collect(cmspecs::kinematics::kDist.scoped(), *controller->kin_proxy());
  collect(cmspecs::spatial::kInterferenceCounts.scoped(),
          *controller->inta_tracker());
  collect_if(cmspecs::spatial::kInterferenceLocs3D.scoped(),
             *controller->inta_tracker(),
             [&](const rmetrics::base_metrics&) {
               return controller->inta_tracker()->exp_interference();
             });
#if defined(COSM_HAL_TARGET_HAS_BATTERY_SENSOR)
  auto battery = controller->saa()->sensing()->battery();
  if (nullptr != battery) {
    collect(cmspecs::sensors::kBattery.scoped(), *battery);
  }
#endif
} /* collect_from_controller() */

void fs_output_manager::collect_from_controller(
    const ccontroller::base_controller3D* const controller) {
  collect(cmspecs::spatial::kDistPosition3D.scoped(), *controller);
  collect(cmspecs::kinematics::kAvg.scoped(), *controller->kin_proxy());
  collect(cmspecs::kinematics::kDist.scoped(), *controller->kin_proxy());
  collect(cmspecs::spatial::kInterferenceCounts.scoped(),
          *controller->inta_tracker());
  collect_if(cmspecs::spatial::kInterferenceLocs3D.scoped(),
             *controller->inta_tracker(),
             [&](const rmetrics::base_metrics&) {
               return controller->inta_tracker()->exp_interference();
             });
#if defined(COSM_HAL_TARGET_HAS_BATTERY_SENSOR)
  auto battery = controller->saa()->sensing()->battery();
  if (nullptr != battery) {
    collect(cmspecs::sensors::kBattery.scoped(), *battery);
  }
#endif
} /* collect_from_controller() */

void fs_output_manager::collect_from_arena(
    const carena::base_arena_map* const map) {
  collect(cmspecs::blocks::kMotion.scoped(), *map->block_motion_handler());
  collect(cmspecs::blocks::kDistributor.scoped(), *map->block_distributor());
  for (auto* cluster : map->block_distributor()->block_clustersro()) {
    collect(cmspecs::blocks::kClusters.scoped(), *cluster);
  } /* for(&cluster..) */
} /* collect_from_arena() */

void fs_output_manager::register_standard(
    const rmconfig::metrics_config* mconfig) {
  using sink_list = rmpl::typelist<
      rmpl::identity<cconvergence::metrics::convergence_metrics_csv_sink>,
      rmpl::identity<cfbd::metrics::distributor_metrics_csv_sink>,
      rmpl::identity<cfmetrics::block_motion_metrics_csv_sink>,
      rmpl::identity<cfsm::metrics::block_transporter_metrics_csv_sink>,
      rmpl::identity<cfmetrics::block_transportee_metrics_csv_sink>,
      rmpl::identity<csmetrics::goal_acq_metrics_csv_sink>,
    rmpl::identity<csmetrics::interference_metrics_csv_sink>,
    rmpl::identity<chsensors::metrics::battery_metrics_csv_sink>,
    rmpl::identity<cssnest::metrics::acq_metrics_csv_sink>,
      rmpl::identity<ctvmetrics::population_dynamics_metrics_csv_sink>,
      rmpl::identity<csmetrics::nest_zone_metrics_csv_sink> >;

  rmetrics::creatable_collector_set creatable_set = {
    { typeid(cconvergence::metrics::convergence_metrics_collector),
      cmspecs::kConvergence.xml(),
      cmspecs::kConvergence.scoped(),
      rmetrics::output_mode::ekAPPEND },
    { typeid(csmetrics::interference_metrics_collector),
      cmspecs::spatial::kInterferenceCounts.xml(),
      cmspecs::spatial::kInterferenceCounts.scoped(),
      rmetrics::output_mode::ekAPPEND },
    { typeid(csmetrics::nest_zone_metrics_collector),
      cmspecs::spatial::kNestZone.xml(),
      cmspecs::spatial::kNestZone.scoped(),
      rmetrics::output_mode::ekAPPEND },
    { typeid(cfbd::metrics::distributor_metrics_collector),
      cmspecs::blocks::kDistributor.xml(),
      cmspecs::blocks::kDistributor.scoped(),
      rmetrics::output_mode::ekAPPEND },
    { typeid(cfmetrics::block_motion_metrics_collector),
      cmspecs::blocks::kMotion.xml(),
      cmspecs::blocks::kMotion.scoped(),
      rmetrics::output_mode::ekAPPEND },
    { typeid(cfsm::metrics::block_transporter_metrics_collector),
      cmspecs::blocks::kTransporter.xml(),
      cmspecs::blocks::kTransporter.scoped(),
      rmetrics::output_mode::ekAPPEND },
    { typeid(chsensors::metrics::battery_metrics_collector),
      cmspecs::sensors::kBattery.xml(),
      cmspecs::sensors::kBattery.scoped(),
      rmetrics::output_mode::ekAPPEND },
    { typeid(cfmetrics::block_transportee_metrics_collector),
      cmspecs::blocks::kTransportee.xml(),
      cmspecs::blocks::kTransportee.scoped(),
      rmetrics::output_mode::ekAPPEND },
    { typeid(csmetrics::goal_acq_metrics_collector),
      cmspecs::blocks::kAcqCounts.xml(),
      cmspecs::blocks::kAcqCounts.scoped(),
      rmetrics::output_mode::ekAPPEND },
    { typeid(cssnest::metrics::acq_metrics_collector),
      cmspecs::strategy::nest::kAcq.xml(),
      cmspecs::strategy::nest::kAcq.scoped(),
      rmetrics::output_mode::ekAPPEND },
    { typeid(ctvmetrics::population_dynamics_metrics_collector),
      cmspecs::tv::kPopulation.xml(),
      cmspecs::tv::kPopulation.scoped(),
      rmetrics::output_mode::ekAPPEND },
  };

  rmetrics::register_with_sink<cargos::metrics::fs_output_manager,
                               rmetrics::file_sink_registerer>
      csv(this, creatable_set);
  rmetrics::register_using_config<decltype(csv), rmconfig::file_sink_config>
      registerer(std::move(csv), &mconfig->csv);
  boost::mpl::for_each<sink_list>(registerer);
} /* register_standard() */

void fs_output_manager::register_with_arena_dims2D(
    const rmconfig::metrics_config* mconfig,
    const rmath::vector2z& dims) {
  using sink_typelist = rmpl::typelist<
      rmpl::identity<csmetrics::dist2D_pos_metrics_csv_sink>,
      rmpl::identity<csmetrics::explore_locs2D_metrics_csv_sink>,
      rmpl::identity<csmetrics::goal_acq_locs2D_metrics_csv_sink>,
      rmpl::identity<csmetrics::interference_locs2D_metrics_csv_sink>,
      rmpl::identity<csmetrics::vector_locs2D_metrics_csv_sink> >;

  rmetrics::creatable_collector_set creatable_set = {
    { typeid(csmetrics::dist2D_pos_metrics_collector),
      cmspecs::spatial::kDistPosition2D.xml(),
      cmspecs::spatial::kDistPosition2D.scoped(),
      rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE },
    { typeid(csmetrics::interference_locs2D_metrics_collector),
      cmspecs::spatial::kInterferenceLocs2D.xml(),
      cmspecs::spatial::kInterferenceLocs2D.scoped(),
      rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE },
    { typeid(csmetrics::explore_locs2D_metrics_collector),
      cmspecs::blocks::kAcqExploreLocs2D.xml(),
      cmspecs::blocks::kAcqExploreLocs2D.scoped(),
      rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE },
    { typeid(csmetrics::goal_acq_locs2D_metrics_collector),
      cmspecs::blocks::kAcqLocs2D.xml(),
      cmspecs::blocks::kAcqLocs2D.scoped(),
      rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE },
    { typeid(csmetrics::vector_locs2D_metrics_collector),
      cmspecs::blocks::kAcqVectorLocs2D.xml(),
      cmspecs::blocks::kAcqVectorLocs2D.scoped(),
      rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE },
  };

  auto extra_args = std::make_tuple(dims);
  rmetrics::register_with_sink<cargos::metrics::fs_output_manager,
                               rmetrics::file_sink_registerer,
                               decltype(extra_args)>
      csv(this, creatable_set, extra_args);
  rmetrics::register_using_config<decltype(csv), rmconfig::file_sink_config>
      registerer(std::move(csv), &mconfig->csv);

  boost::mpl::for_each<sink_typelist>(registerer);
} /* register_with_arena_dims2D() */

void fs_output_manager::register_with_arena_dims3D(
    const rmconfig::metrics_config* mconfig,
    const rmath::vector3z& dims) {
  using sink_typelist = rmpl::typelist<
      rmpl::identity<csmetrics::dist3D_pos_metrics_csv_sink>,
      rmpl::identity<csmetrics::explore_locs3D_metrics_csv_sink>,
      rmpl::identity<csmetrics::interference_locs3D_metrics_csv_sink>,
      rmpl::identity<csmetrics::vector_locs3D_metrics_csv_sink> >;

  rmetrics::creatable_collector_set creatable_set = {
    { typeid(csmetrics::dist3D_pos_metrics_collector),
      cmspecs::spatial::kDistPosition3D.xml(),
      cmspecs::spatial::kDistPosition3D.scoped(),
      rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE },
    { typeid(csmetrics::interference_locs3D_metrics_collector),
      cmspecs::spatial::kInterferenceLocs3D.xml(),
      cmspecs::spatial::kInterferenceLocs3D.scoped(),
      rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE },
    { typeid(csmetrics::explore_locs3D_metrics_collector),
      cmspecs::blocks::kAcqExploreLocs3D.xml(),
      cmspecs::blocks::kAcqExploreLocs3D.scoped(),
      rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE },
    { typeid(csmetrics::vector_locs3D_metrics_collector),
      cmspecs::blocks::kAcqVectorLocs3D.xml(),
      cmspecs::blocks::kAcqVectorLocs3D.scoped(),
      rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE },
  };

  auto extra_args = std::make_tuple(dims);
  rmetrics::register_with_sink<cargos::metrics::fs_output_manager,
                               rmetrics::file_sink_registerer,
                               decltype(extra_args)>
      csv(this, creatable_set, extra_args);
  rmetrics::register_using_config<decltype(csv), rmconfig::file_sink_config>
      registerer(std::move(csv), &mconfig->csv);

  boost::mpl::for_each<sink_typelist>(registerer);
} /* register_with_arena_dims3D() */

void fs_output_manager::register_with_n_block_clusters(
    const rmconfig::metrics_config* mconfig,
    size_t n_clusters) {
  using sink_typelist =
      rmpl::typelist<rmpl::identity<cfmetrics::block_cluster_metrics_csv_sink> >;

  rmetrics::creatable_collector_set creatable_set = {
    { typeid(cfmetrics::block_cluster_metrics_collector),
      cmspecs::blocks::kClusters.xml(),
      cmspecs::blocks::kClusters.scoped(),
      rmetrics::output_mode::ekAPPEND }
  };
  auto extra_args = std::make_tuple(n_clusters);
  rmetrics::register_with_sink<cargos::metrics::fs_output_manager,
                               rmetrics::file_sink_registerer,
                               decltype(extra_args)>
      csv(this, creatable_set, extra_args);
  rmetrics::register_using_config<decltype(csv), rmconfig::file_sink_config>
      registerer(std::move(csv), &mconfig->csv);

  boost::mpl::for_each<sink_typelist>(registerer);
} /* register_with_n_block_clusters() */

void fs_output_manager::register_kinematics(
    const rmconfig::metrics_config* mconfig,
    size_t n_robots) {
  using sink_typelist =
      rmpl::typelist<rmpl::identity<ckmetrics::kinematics_metrics_avg_csv_sink>,
      rmpl::identity<ckmetrics::kinematics_metrics_dist_csv_sink>>;

  rmetrics::creatable_collector_set creatable_set = {
        { typeid(ckmetrics::kinematics_metrics_collector),
      cmspecs::kinematics::kAvg.xml(),
      cmspecs::kinematics::kAvg.scoped(),
      rmetrics::output_mode::ekAPPEND,
      typeid(ckmetrics::kinematics_metrics_avg_csv_sink)},
        { typeid(ckmetrics::kinematics_metrics_collector),
      cmspecs::kinematics::kDist.xml(),
      cmspecs::kinematics::kDist.scoped(),
      rmetrics::output_mode::ekAPPEND,
      typeid(ckmetrics::kinematics_metrics_dist_csv_sink)},
  };
  auto extra_args = std::make_tuple(n_robots, ckmetrics::kContexts.size());
  rmetrics::register_with_sink<cargos::metrics::fs_output_manager,
                               rmetrics::file_sink_registerer,
                               decltype(extra_args)>
      csv(this, creatable_set, extra_args);
  rmetrics::register_using_config<decltype(csv), rmconfig::file_sink_config>
      registerer(std::move(csv), &mconfig->csv);

  boost::mpl::for_each<sink_typelist>(registerer);
} /* register_kinematics() */

} /* namespace cosm::argos::metrics */
