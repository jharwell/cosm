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
#include "cosm/ros/metrics/registrable.hpp"


#include "cosm/foraging/metrics/block_cluster_metrics_collector.hpp"
#include "cosm/foraging/metrics/block_cluster_metrics_csv_sink.hpp"
#include "cosm/foraging/metrics/block_transportee_metrics_collector.hpp"
#include "cosm/foraging/metrics/block_transportee_metrics_csv_sink.hpp"
#include "cosm/fsm/metrics/block_transporter_metrics_collector.hpp"
#include "cosm/fsm/metrics/block_transporter_metrics_csv_sink.hpp"
#include "cosm/metrics/specs.hpp"
#include "cosm/spatial/metrics/interference_metrics_collector.hpp"
#include "cosm/spatial/metrics/interference_metrics_csv_sink.hpp"
#include "cosm/kin/metrics/kinematics_metrics.hpp"
#include "cosm/kin/metrics/kinematics_metrics_collector.hpp"
#include "cosm/kin/metrics/kinematics_metrics_dist_csv_sink.hpp"
#include "cosm/kin/metrics/kinematics_metrics_avg_csv_sink.hpp"
#include "cosm/hal/sensors/metrics/battery_metrics_collector.hpp"
#include "cosm/hal/sensors/metrics/battery_metrics_csv_sink.hpp"

#include "rcppsw/utils/maskable_enum.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ros::metrics::registrable {

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
rmetrics::creatable_collector_set kStandard = {
  { typeid(csmetrics::interference_metrics_collector),
    cmspecs::spatial::kInterferenceCounts.xml(),
    cmspecs::spatial::kInterferenceCounts.scoped(),
    rmetrics::output_mode::ekSTREAM | rmetrics::output_mode::ekAPPEND,
    typeid(csmetrics::interference_metrics_csv_sink)},
  { typeid(cfsm::metrics::block_transporter_metrics_collector),
    cmspecs::blocks::kTransporter.xml(),
    cmspecs::blocks::kTransporter.scoped(),
    rmetrics::output_mode::ekSTREAM | rmetrics::output_mode::ekAPPEND,
    typeid(cfsm::metrics::block_transporter_metrics_csv_sink)},
  { typeid(cfmetrics::block_transportee_metrics_collector),
    cmspecs::blocks::kTransportee.xml(),
    cmspecs::blocks::kTransportee.scoped(),
    rmetrics::output_mode::ekSTREAM | rmetrics::output_mode::ekAPPEND ,
    typeid(cfmetrics::block_transportee_metrics_csv_sink)},
  { typeid(chsensors::metrics::battery_metrics_collector),
    cmspecs::sensors::kBattery.xml(),
    cmspecs::sensors::kBattery.scoped(),
    rmetrics::output_mode::ekSTREAM | rmetrics::output_mode::ekAPPEND,
    typeid(chsensors::metrics::battery_metrics_csv_sink)},
};

rmetrics::creatable_collector_set kWithNRobots = {
  { typeid(ckin::metrics::kinematics_metrics_collector),
    cmspecs::kinematics::kAvg.xml(),
    cmspecs::kinematics::kAvg.scoped(),
    rmetrics::output_mode::ekSTREAM | rmetrics::output_mode::ekAPPEND,
    typeid(ckin::metrics::kinematics_metrics_avg_csv_sink),},
  { typeid(ckin::metrics::kinematics_metrics_collector),
    cmspecs::kinematics::kDist.xml(),
    cmspecs::kinematics::kDist.scoped(),
    rmetrics::output_mode::ekSTREAM | rmetrics::output_mode::ekAPPEND,
    typeid(ckin::metrics::kinematics_metrics_dist_csv_sink)},
};

rmetrics::creatable_collector_set kWithNBlockClusters = {
  { typeid(cfmetrics::block_cluster_metrics_collector),
    cmspecs::blocks::kClusters.xml(),
    cmspecs::blocks::kClusters.scoped(),
    rmetrics::output_mode::ekSTREAM | rmetrics::output_mode::ekAPPEND,
    typeid(cfmetrics::block_cluster_metrics_csv_sink)}
};

} /* namespace cosm::ros::metrics::registrable */
