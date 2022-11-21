/**
 * \file specs.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/metrics/name_spec.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::metrics::specs {

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
extern name_spec kConvergence;

namespace kinematics {

extern name_spec kAvg;
extern name_spec kDist;

} /* namespace kinematics */

namespace spatial {

extern name_spec kInterferenceCounts;
extern name_spec kInterferenceLocs2D;
extern name_spec kInterferenceLocs3D;
extern name_spec kNestZone;
extern name_spec kDistPosition2D;
extern name_spec kDistPosition3D;

} /* namespace spatial */

namespace sensors {
extern name_spec kBattery;
} /* namespace sensors */

namespace blocks {

extern name_spec kDistributor;
extern name_spec kMotion;
extern name_spec kClusters;
extern name_spec kTransporter;
extern name_spec kTransportee;
extern name_spec kAcqCounts;
extern name_spec kAcqExploreLocs2D;
extern name_spec kAcqLocs2D;
extern name_spec kAcqExploreLocs3D;
extern name_spec kAcqVectorLocs2D;
extern name_spec kAcqVectorLocs3D;

} /* namespace blocks */

namespace strategy {
namespace nest {

extern name_spec kAcq;

} /* namespace strategy */

} /* namespace strategy */

namespace tv {

extern name_spec kPopulation;
extern name_spec kEnvironment;

} /* namespace tv */

namespace tasks {

extern name_spec kDistribution;

} /* namespace tasks */

} /* namespace cosm::metrics::specs */
