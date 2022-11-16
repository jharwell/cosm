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
NS_START(cosm, metrics, specs);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
extern name_spec kConvergence;

NS_START(spatial);

extern name_spec kMovement;
extern name_spec kInterferenceCounts;
extern name_spec kInterferenceLocs2D;
extern name_spec kInterferenceLocs3D;
extern name_spec kNestZone;
extern name_spec kDistPosition2D;
extern name_spec kDistPosition3D;

NS_END(spatial);

NS_START(sensors);
extern name_spec kBattery;
NS_END(sensors);

NS_START(blocks);

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

NS_END(blocks);

NS_START(strategy);
NS_START(nest);

extern name_spec kAcq;

NS_END(strategy);

NS_END(strategy);

NS_START(tv);

extern name_spec kPopulation;
extern name_spec kEnvironment;

NS_END(tv);

NS_START(tasks);

extern name_spec kDistribution;

NS_END(tasks);

NS_END(specs, metrics, cosm);
