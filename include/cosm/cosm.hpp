/**
 * \file cosm.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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

#include "cosm/config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
/**
 * It is better to define the namespace aliases exported by COSM here, rather
 * than having them be multiply defined in different downstream projects (DRY
 * FTW!).
 *
 * Convention: Namespace aliases from cosm all start with \c 'c', and the first
 * letter of all nested namespaces except the innermost one should be included
 * before the innermost. For example, cosm::fsm::metrics should have the \c 'c'
 * from \c 'cosm' and the \c 'f' from 'fsm' before the target namespace \c
 * 'metrics'.
 */

/**
 * \namespace cosm
 * \brief The root namespace of the cosm library.
 */
namespace cosm {

/**
 * \namespace tv
 * \brief Temporal variance applicators.
 */
namespace tv {
namespace metrics {}
} /* namespace tv */

/**
 * \namespace subsystem
 * \brief Robotic susbystems.
 */

namespace subsystem {
namespace perception {
namespace config {}
} /* namespace perception */
} /* namespace subsystem */

/**
 * \namespace hal
 * \brief Hardware Abstraction Layer
 */
namespace hal {}

/**
 * \namespace kin2D
 * \brief 2D kinematics for wheeled robots.
 */
namespace kin2D {}

/**
 * \namespace steer2D
 * \brief Steering forces for 2D wheeled robots.
 */
namespace steer2D {}

/**
 * \namespace convergence
 * \brief Swarm convergence measures and calculators.
 */
namespace convergence {
namespace config {}
}

/**
 * \namespace metrics
 * \brief Reusable metric interfaces and collectors for robotics.
 */
namespace metrics {
namespace config {}
} /* namespace metrics */

/**
 * \namespace repr
 * \brief Representation of STUFF that is not strictly a data structure,
 *        mathematical construct, etc., but that is used to model things in the
 *        real world.
 */
namespace repr {
namespace config {}
namespace operations {}
} /* namespace repr */

/**
 * \namespace controller
 * \brief Robot controllers.
 */
namespace controller {

namespace operations {}
namespace metrics {}
namespace config {}
} /* namespace controller */

/**
 * \namespace robots
 * \brief Adaptor layer containing specializations/extensions of controllers,
 *        subsystems, etc., that are specific to a particular robot model.
 */
namespace robots {

/**
 * \namespace footbot
 * \brief Adaptation layer to the ARGoS footbot robot.
 */
namespace footbot {}
} /* namespace robots */

/**
 * \namespace fsm
 * \brief Reusable FSMs not specific to any particular application within swarm
 *        robotics.
 */
namespace fsm {}

/**
 * \namespace vis vis
 * \brief Qt visualization bindings.
 */
namespace vis {
namespace config {}
} /* namespace vis */

namespace ds {
namespace config {}
namespace operations {}
} /* namespace ds */

namespace pal {
namespace config {}
namespace operations {}
namespace tv {}
} /* namespace pal */

namespace ta {
namespace metrics {}
} /* namespace ta */

namespace spatial {
namespace fsm {}
namespace metrics {}
namespace expstrat {}
} /* namespace spatial */

namespace foraging {
namespace config {}
namespace block_dist {}
namespace repr {}
namespace ds {}
namespace operations {}
namespace utils {}
namespace oracle {}
namespace metrics {}
} /* namespace foraging */

namespace arena {
namespace operations {}
namespace ds {}
namespace repr {}
namespace metrics {}
namespace config {}

} /* namespace arena */

namespace oracle {
namespace config {}
} /* namespace oracle */

namespace interactors {}
} /* namespace cosm */

/** @} */

namespace csubsystem = cosm::subsystem;
namespace csperception = csubsystem::perception;
namespace cspconfig = csperception::config;

namespace cinteractors = cosm::interactors;
namespace ccontroller = cosm::controller;
namespace ccontconfig = ccontroller::config;
namespace ccops = ccontroller::operations;
namespace ccmetrics = ccontroller::metrics;

namespace cforaging = cosm::foraging;
namespace cfrepr = cforaging::repr;
namespace cforacle = cforaging::oracle;
namespace cfutils = cforaging::utils;
namespace cfds = cforaging::ds;
namespace cfops = cforaging::operations;
namespace cfconfig = cforaging::config;
namespace cfbd = cforaging::block_dist;
namespace cfmetrics = cforaging::metrics;

namespace cspatial = cosm::spatial;
namespace csfsm = cspatial::fsm;
namespace csmetrics = cspatial::metrics;
namespace csexpstrat = cspatial::expstrat;

namespace carena = cosm::arena;
namespace cads = carena::ds;
namespace caconfig = carena::config;
namespace cametrics = carena::metrics;
namespace carepr = carena::repr;
namespace caops = carena::operations;

namespace cconvergence = cosm::convergence;
namespace cconvconfig = cconvergence::config;

namespace cta = cosm::ta;
namespace ctametrics = cta::metrics;

namespace ctv = cosm::tv;
namespace ctvmetrics = ctv::metrics;

namespace cmetrics = cosm::metrics;
namespace cmconfig = cmetrics::config;

namespace cvis = cosm::vis;
namespace cvconfig = cvis::config;

namespace cds = cosm::ds;
namespace cdconfig = cds::config;
namespace cdops = cds::operations;

namespace cpal = cosm::pal;
namespace cpops = cpal::operations;
namespace cpconfig = cpal::config;
namespace cptv = cpal::tv;

namespace coracle = cosm::oracle;
namespace coconfig = coracle::config;

namespace crepr = cosm::repr;
namespace crops = crepr::operations;

namespace crconfig = crepr::config;

namespace cfsm = cosm::fsm;
namespace chal = cosm::hal;
namespace ckin2D = cosm::kin2D;
namespace csteer2D = cosm::steer2D;
namespace crobots = cosm::robots;
namespace crfootbot = crobots::footbot;

#endif /* INCLUDE_COSM_COSM_HPP_ */
