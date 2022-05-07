/**
 * \file specs.cpp
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
#include "cosm/metrics/specs.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, metrics, specs);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
name_spec kConvergence = { "swarm_convergence", "swarm/convergence" };

NS_START(spatial);
name_spec kMovement = { "spatial_movement", "spatial/movement" };
name_spec kInterferenceCounts = { "spatial_interference_counts",
                                  "spatial/interference/counts" };
name_spec kInterferenceLocs2D = { "spatial_interference_locs2D",
                                  "spatial/interference/locs2D" };
name_spec kInterferenceLocs3D = { "spatial_interference_locs3D",
                                  "spatial/interference/locs3D" };
name_spec kNestZone = { "spatial_nest_zone", "spatial/nest_zone" };
name_spec kDistPosition2D = { "spatial_dist_pos2D", "spatial/dist/pos2D" };
name_spec kDistPosition3D = { "spatial_dist_pos3D", "spatial/dist/pos3D" };
NS_END(spatial);

NS_START(blocks);

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

NS_END(blocks);

NS_START(strategy);

NS_START(nest);

name_spec kAcq = { "nest_acq_strategy", "strategy/nest/acq" };

NS_END(nest);

NS_END(strategy);


NS_START(tv);
name_spec kPopulation = { "tv_population", "tv/population" };
name_spec kEnvironment = { "tv_environment", "tv/environment" };

NS_END(tv);

NS_START(tasks);

name_spec kDistribution = { "task_distribution", "tasks/distribution" };

NS_END(tasks);

NS_END(specs, metrics, cosm);
