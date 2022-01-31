/**
 * \file specs.hpp
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

#ifndef INCLUDE_COSM_METRICS_SPECS_HPP_
#define INCLUDE_COSM_METRICS_SPECS_HPP_

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

NS_START(blocks);

extern name_spec kDistributor;
extern name_spec kMotion;
extern name_spec kClusters;
extern name_spec kTransporter;
extern name_spec kTransportee;
extern name_spec kManipulation;
extern name_spec kAcqCounts;
extern name_spec kAcqExploreLocs2D;
extern name_spec kAcqLocs2D;
extern name_spec kAcqExploreLocs3D;
extern name_spec kAcqVectorLocs2D;
extern name_spec kAcqVectorLocs3D;

NS_END(blocks);

NS_START(strategy);

extern name_spec kNestAcq;

NS_END(strategy);

NS_START(tv);

extern name_spec kPopulation;
extern name_spec kEnvironment;

NS_END(tv);

NS_START(tasks);

extern name_spec kDistribution;

NS_END(tasks);

NS_END(specs, metrics, cosm);

#endif /* INCLUDE_COSM_METRICS_SPECS_HPP_ */
