/**
 * \file registrable.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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
#include "cosm/ros/metrics/registrable.hpp"

#include "rcppsw/utils/maskable_enum.hpp"

#include "cosm/foraging/metrics/block_cluster_metrics_collector.hpp"
#include "cosm/fsm/metrics/block_transporter_metrics_collector.hpp"
#include "cosm/foraging/metrics/block_transportee_metrics_collector.hpp"
#include "cosm/spatial/metrics/interference_metrics_collector.hpp"
#include "cosm/spatial/metrics/movement_metrics.hpp"
#include "cosm/spatial/metrics/movement_metrics_collector.hpp"
#include "cosm/metrics/specs.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ros, metrics, registrable);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
rmetrics::creatable_collector_set kStandard = {
  { typeid(csmetrics::movement_metrics_collector),
    cmspecs::spatial::kMovement.xml(),
    cmspecs::spatial::kMovement.scoped(),
    rmetrics::output_mode::ekSTREAM | rmetrics::output_mode::ekAPPEND },
  { typeid(csmetrics::interference_metrics_collector),
    cmspecs::spatial::kInterferenceCounts.xml(),
    cmspecs::spatial::kInterferenceCounts.scoped(),
    rmetrics::output_mode::ekSTREAM | rmetrics::output_mode::ekAPPEND},
  { typeid(cfsm::metrics::block_transporter_metrics_collector),
    cmspecs::blocks::kTransporter.xml(),
    cmspecs::blocks::kTransporter.scoped(),
    rmetrics::output_mode::ekSTREAM | rmetrics::output_mode::ekAPPEND},
  { typeid(cfmetrics::block_transportee_metrics_collector),
    cmspecs::blocks::kTransportee.xml(),
    cmspecs::blocks::kTransportee.scoped(),
    rmetrics::output_mode::ekSTREAM | rmetrics::output_mode::ekAPPEND},
};

rmetrics::creatable_collector_set kWithNBlockClusters = {
  { typeid(cfmetrics::block_cluster_metrics_collector),
    cmspecs::blocks::kClusters.xml(),
    cmspecs::blocks::kClusters.scoped(),
    rmetrics::output_mode::ekSTREAM | rmetrics::output_mode::ekAPPEND}
};

NS_END(registrable, metrics, ros, cosm);
