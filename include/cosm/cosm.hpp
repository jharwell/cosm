/**
 * @file cosm.hpp
 *
 * @copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_COSM_HPP_
#define INCLUDE_COSM_COSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/rcppsw.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm {
namespace tv {}
namespace subsystem {}
namespace hal {}
namespace kin2D {}
namespace steer2D {}
namespace convergence {}
namespace metrics {}
namespace repr {}
namespace controller {}
namespace robots {
namespace footbot {}
} /* namespace robots */

namespace fsm {
namespace metrics {}
} /* namespace fsm */

} /* namespace cosm */

/*
 * It is better to define the namespace aliases exported by COSM here, rather
 * than having them be multiply defined in different downstream projects (DRY
 * FTW!).
 *
 * Convention: Namespace aliases from cosm all start with 'c', and the first
 * letter of all nested namespaces except the innermost one should be included
 * before the innermost. For example, cosm::fsm::metrics should have the
 * 'c' from 'cosm' and the 'f' from 'fsm' before the target namespace
 * 'metrics'.
 */

namespace csubsystem = cosm::subsystem;
namespace ccontroller = cosm::controller;
namespace cfsm = cosm::fsm;
namespace cfmetrics = cfsm::metrics;
namespace chal = cosm::hal;
namespace ckin2D = cosm::kin2D;
namespace csteer2D = cosm::steer2D;
namespace crobots = cosm::robots;
namespace crfootbot = crobots::footbot;
namespace ctv = cosm::tv;
namespace cconvergence = cosm::convergence;
namespace cmetrics = cosm::metrics;
namespace ctv = cosm::tv;
namespace crepr = cosm::repr;

#endif /* INCLUDE_COSM_COSM_HPP_ */
