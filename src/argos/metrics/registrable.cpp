/**
 * \file registrable.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/argos/metrics/registrable.hpp"

#include "cosm/convergence/metrics/convergence_metrics_collector.hpp"
#include "cosm/convergence/metrics/convergence_metrics_csv_sink.hpp"
#include "cosm/spatial/metrics/interference_metrics_collector.hpp"
#include "cosm/spatial/metrics/interference_metrics_csv_sink.hpp"
#include "cosm/foraging/metrics/block_cluster_metrics_collector.hpp"
#include "cosm/foraging/metrics/block_cluster_metrics_csv_sink.hpp"
#include "cosm/foraging/metrics/block_transportee_metrics_collector.hpp"
#include "cosm/foraging/metrics/block_transportee_metrics_csv_sink.hpp"
#include "cosm/fsm/metrics/block_transporter_metrics_collector.hpp"
#include "cosm/fsm/metrics/block_transporter_metrics_csv_sink.hpp"
#include "cosm/metrics/specs.hpp"
#include "cosm/kin/metrics/kinematics_metrics.hpp"
#include "cosm/kin/metrics/kinematics_metrics_collector.hpp"
#include "cosm/kin/metrics/kinematics_metrics_avg_csv_sink.hpp"
#include "cosm/kin/metrics/kinematics_metrics_dist_csv_sink.hpp"
#include "cosm/spatial/metrics/nest_zone_metrics_collector.hpp"
#include "cosm/spatial/metrics/nest_zone_metrics_csv_sink.hpp"
#include "cosm/foraging/block_dist/metrics/distributor_metrics_collector.hpp"
#include "cosm/foraging/block_dist/metrics/distributor_metrics_csv_sink.hpp"
#include "cosm/foraging/metrics/block_motion_metrics_collector.hpp"
#include "cosm/foraging/metrics/block_motion_metrics_csv_sink.hpp"
#include "cosm/hal/sensors/metrics/battery_metrics_collector.hpp"
#include "cosm/hal/sensors/metrics/battery_metrics_csv_sink.hpp"
#include "cosm/spatial/metrics/goal_acq_metrics_collector.hpp"
#include "cosm/spatial/metrics/goal_acq_metrics_csv_sink.hpp"
#include "cosm/spatial/strategy/nest/metrics/acq_metrics_collector.hpp"
#include "cosm/spatial/strategy/nest/metrics/acq_metrics_csv_sink.hpp"
#include "cosm/tv/metrics/population_dynamics_metrics_collector.hpp"
#include "cosm/tv/metrics/population_dynamics_metrics_csv_sink.hpp"

#include "cosm/spatial/metrics/vector_locs2D_metrics_collector.hpp"
#include "cosm/spatial/metrics/vector_locs2D_metrics_csv_sink.hpp"
#include "cosm/spatial/metrics/dist2D_pos_metrics_collector.hpp"
#include "cosm/spatial/metrics/dist2D_pos_metrics_csv_sink.hpp"
#include "cosm/spatial/metrics/explore_locs2D_metrics_collector.hpp"
#include "cosm/spatial/metrics/explore_locs2D_metrics_csv_sink.hpp"
#include "cosm/spatial/metrics/interference_locs2D_metrics_collector.hpp"
#include "cosm/spatial/metrics/interference_locs2D_metrics_csv_sink.hpp"
#include "cosm/spatial/metrics/goal_acq_locs2D_metrics_collector.hpp"
#include "cosm/spatial/metrics/goal_acq_locs2D_metrics_csv_sink.hpp"

#include "cosm/spatial/metrics/dist3D_pos_metrics_collector.hpp"
#include "cosm/spatial/metrics/dist3D_pos_metrics_csv_sink.hpp"
#include "cosm/spatial/metrics/explore_locs3D_metrics_collector.hpp"
#include "cosm/spatial/metrics/explore_locs3D_metrics_csv_sink.hpp"
#include "cosm/spatial/metrics/interference_locs3D_metrics_collector.hpp"
#include "cosm/spatial/metrics/interference_locs3D_metrics_csv_sink.hpp"
#include "cosm/spatial/metrics/vector_locs3D_metrics_collector.hpp"
#include "cosm/spatial/metrics/vector_locs3D_metrics_csv_sink.hpp"


#include "rcppsw/utils/maskable_enum.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::argos::metrics::registrable {

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
rmetrics::creatable_collector_set kStandard = {
      { typeid(cconvergence::metrics::convergence_metrics_collector),
      cmspecs::kConvergence.xml(),
      cmspecs::kConvergence.scoped(),
      rmetrics::output_mode::ekAPPEND,
      typeid(cconvergence::metrics::convergence_metrics_csv_sink)},
    { typeid(csmetrics::interference_metrics_collector),
      cmspecs::spatial::kInterferenceCounts.xml(),
      cmspecs::spatial::kInterferenceCounts.scoped(),
      rmetrics::output_mode::ekAPPEND,
      typeid(csmetrics::interference_metrics_csv_sink)},
    { typeid(csmetrics::nest_zone_metrics_collector),
      cmspecs::spatial::kNestZone.xml(),
      cmspecs::spatial::kNestZone.scoped(),
      rmetrics::output_mode::ekAPPEND,
      typeid(csmetrics::nest_zone_metrics_csv_sink)},
    { typeid(cfbd::metrics::distributor_metrics_collector),
      cmspecs::blocks::kDistributor.xml(),
      cmspecs::blocks::kDistributor.scoped(),
      rmetrics::output_mode::ekAPPEND,
      typeid(cfbd::metrics::distributor_metrics_csv_sink),},
    { typeid(cfmetrics::block_motion_metrics_collector),
      cmspecs::blocks::kMotion.xml(),
      cmspecs::blocks::kMotion.scoped(),
      rmetrics::output_mode::ekAPPEND,
      typeid(cfmetrics::block_motion_metrics_csv_sink),},
    { typeid(cfsm::metrics::block_transporter_metrics_collector),
      cmspecs::blocks::kTransporter.xml(),
      cmspecs::blocks::kTransporter.scoped(),
      rmetrics::output_mode::ekAPPEND,
      typeid(cfsm::metrics::block_transporter_metrics_csv_sink)},
    { typeid(chsensors::metrics::battery_metrics_collector),
      cmspecs::sensors::kBattery.xml(),
      cmspecs::sensors::kBattery.scoped(),
      rmetrics::output_mode::ekAPPEND,
      typeid(chsensors::metrics::battery_metrics_csv_sink)},
    { typeid(cfmetrics::block_transportee_metrics_collector),
      cmspecs::blocks::kTransportee.xml(),
      cmspecs::blocks::kTransportee.scoped(),
      rmetrics::output_mode::ekAPPEND,
      typeid(cfmetrics::block_transportee_metrics_csv_sink)},
    { typeid(csmetrics::goal_acq_metrics_collector),
      cmspecs::blocks::kAcqCounts.xml(),
      cmspecs::blocks::kAcqCounts.scoped(),
      rmetrics::output_mode::ekAPPEND,
      typeid(csmetrics::goal_acq_metrics_csv_sink),},
    { typeid(cssnest::metrics::acq_metrics_collector),
      cmspecs::strategy::nest::kAcq.xml(),
      cmspecs::strategy::nest::kAcq.scoped(),
      rmetrics::output_mode::ekAPPEND,
      typeid(cssnest::metrics::acq_metrics_csv_sink),},
    { typeid(ctvmetrics::population_dynamics_metrics_collector),
      cmspecs::tv::kPopulation.xml(),
      cmspecs::tv::kPopulation.scoped(),
      rmetrics::output_mode::ekAPPEND,
      typeid(ctvmetrics::population_dynamics_metrics_csv_sink)}
};

rmetrics::creatable_collector_set kWithArenaDims2D = {
  { typeid(csmetrics::dist2D_pos_metrics_collector),
      cmspecs::spatial::kDistPosition2D.xml(),
      cmspecs::spatial::kDistPosition2D.scoped(),
      rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE,
      typeid(csmetrics::dist2D_pos_metrics_csv_sink)},
  { typeid(csmetrics::interference_locs2D_metrics_collector),
      cmspecs::spatial::kInterferenceLocs2D.xml(),
      cmspecs::spatial::kInterferenceLocs2D.scoped(),
      rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE,
      typeid(csmetrics::interference_locs2D_metrics_csv_sink)},
  { typeid(csmetrics::explore_locs2D_metrics_collector),
      cmspecs::blocks::kAcqExploreLocs2D.xml(),
      cmspecs::blocks::kAcqExploreLocs2D.scoped(),
      rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE,
      typeid(csmetrics::explore_locs2D_metrics_csv_sink)},
  { typeid(csmetrics::goal_acq_locs2D_metrics_collector),
      cmspecs::blocks::kAcqLocs2D.xml(),
      cmspecs::blocks::kAcqLocs2D.scoped(),
      rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE,
      typeid(csmetrics::goal_acq_locs2D_metrics_csv_sink)},
  { typeid(csmetrics::vector_locs2D_metrics_collector),
      cmspecs::blocks::kAcqVectorLocs2D.xml(),
      cmspecs::blocks::kAcqVectorLocs2D.scoped(),
      rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE,
      typeid(csmetrics::vector_locs2D_metrics_csv_sink) },
};
rmetrics::creatable_collector_set kWithArenaDims3D = {
  { typeid(csmetrics::dist3D_pos_metrics_collector),
      cmspecs::spatial::kDistPosition3D.xml(),
      cmspecs::spatial::kDistPosition3D.scoped(),
      rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE,
      typeid(csmetrics::dist3D_pos_metrics_csv_sink),},
  { typeid(csmetrics::interference_locs3D_metrics_collector),
      cmspecs::spatial::kInterferenceLocs3D.xml(),
      cmspecs::spatial::kInterferenceLocs3D.scoped(),
      rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE,
      typeid(csmetrics::interference_locs3D_metrics_csv_sink)},
  { typeid(csmetrics::explore_locs3D_metrics_collector),
      cmspecs::blocks::kAcqExploreLocs3D.xml(),
      cmspecs::blocks::kAcqExploreLocs3D.scoped(),
      rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE,
      typeid(csmetrics::explore_locs3D_metrics_csv_sink)},
  { typeid(csmetrics::vector_locs3D_metrics_collector),
      cmspecs::blocks::kAcqVectorLocs3D.xml(),
      cmspecs::blocks::kAcqVectorLocs3D.scoped(),
      rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE,
      typeid(csmetrics::vector_locs3D_metrics_csv_sink)},
};

rmetrics::creatable_collector_set kKinematics = {
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

rmetrics::creatable_collector_set kWithNBlockClusters = {
  { typeid(cfmetrics::block_cluster_metrics_collector),
    cmspecs::blocks::kClusters.xml(),
    cmspecs::blocks::kClusters.scoped(),
    rmetrics::output_mode::ekSTREAM | rmetrics::output_mode::ekAPPEND,
    typeid(cfmetrics::block_cluster_metrics_csv_sink)}
};

} /* namespace cosm::argos::metrics::registrable */
