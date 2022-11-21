/**
 * \file specs.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/metrics/specs.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::metrics::specs {

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
name_spec kConvergence = { "swarm_convergence", "swarm/convergence" };

namespace kinematics {

name_spec kAvg = { "kinematics_avg", "kinematics/avg" };
name_spec kDist = { "kinematics_dist", "kinematics/dist" };

} /* namespace kinematics */

namespace spatial {
name_spec kInterferenceCounts = { "spatial_interference_counts",
                                  "spatial/interference/counts" };
name_spec kInterferenceLocs2D = { "spatial_interference_locs2D",
                                  "spatial/interference/locs2D" };
name_spec kInterferenceLocs3D = { "spatial_interference_locs3D",
                                  "spatial/interference/locs3D" };
name_spec kNestZone = { "spatial_nest_zone", "spatial/nest_zone" };
name_spec kDistPosition2D = { "spatial_dist_pos2D", "spatial/dist/pos2D" };
name_spec kDistPosition3D = { "spatial_dist_pos3D", "spatial/dist/pos3D" };
} /* namespace spatial */

namespace sensors {
name_spec kBattery = { "battery_state", "sensors/battery"};
} /* namespace sensors */

namespace blocks {

name_spec kDistributor = { "block_distributor", "blocks/distributor" };
name_spec kMotion = { "block_motion", "blocks/motion" };
name_spec kClusters = { "block_clusters", "blocks/clusters" };
name_spec kTransporter = { "block_transporter", "blocks/transporter" };
name_spec kTransportee = { "block_transportee", "blocks/transportee" };
name_spec kAcqCounts = { "block_acq_counts", "blocks/acq/counts" };
name_spec kAcqExploreLocs2D = {
  "block_acq_explore_locs2D",
  "blocks/acq/explore_locs2D",
};
name_spec kAcqLocs2D = {
  "block_acq_locs2D",
  "blocks/acq/locs2D",
};
name_spec kAcqExploreLocs3D = {
  "block_acq_explore_locs3D",
  "blocks/acq/explore_locs3D",
};
name_spec kAcqVectorLocs2D = {
  "block_acq_vector_locs2D",
  "blocks/acq/vector_locs2D",
};
name_spec kAcqVectorLocs3D = {
  "block_acq_vector_locs3D",
  "blocks/acq/vector_locs3D",
};

} /* namespace blocks */

namespace strategy {

namespace nest {

name_spec kAcq = { "nest_acq_strategy", "strategy/nest/acq" };

} /* namespace nest */

} /* namespace strategy */


namespace tv {
name_spec kPopulation = { "tv_population", "tv/population" };
name_spec kEnvironment = { "tv_environment", "tv/environment" };

} /* namespace tv */

namespace tasks {

name_spec kDistribution = { "task_distribution", "tasks/distribution" };

} /* namespace tasks */

} /* namespace cosm::metrics::specs */
