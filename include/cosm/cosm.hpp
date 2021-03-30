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
 *
 * \brief The root namespace of the cosm library.
 */
namespace cosm {

/**
 * \namespace tv
 *
 * \brief Container namespace for temporal variance machinery and
 * configuration. Used to modify swarm behavior over time (e.g., forcing robots
 * to slow down after N timesteps, simulating changing weather conditions).
 */
namespace tv {

/**
 * \namespace metrics
 *
 * \brief Metrics for temporal variance machinery.
 */
namespace metrics {}
} /* namespace tv */

/**
 * \namespace subsystem
 *
 * \brief Classes for robotic susbystems such as sensing, perception, and
 * actuation, and their configuration. Not tied to a specific robot.
 */
namespace subsystem {
namespace perception {
namespace config {}
} /* namespace perception */
} /* namespace subsystem */

/**
 * \namespace hal
 *
 * \brief Hardware Abstraction Layer. Compile time switching between different
 * implementations of robot sensors and actuators to (1) eliminate run-time
 * penalties when de-muxing, (2) present a uniform interface to higher level
 * modules, such as \ref subystem classes, regardless of which robot platform
 * COSM is targeting.
 */
namespace hal {}

/**
 * \namespace kin2D
 *
 * \brief Differential drive classes for wheeled robots moving in 2D, and their
 * configuration.
 */
namespace kin2D {}

/**
 * \namespace steer2D
 *
 * \brief Steering forces for 2D wheeled robots, inspired from Arkin's original
 * 1987 paper: arrival, avoidance, wander, etc., and their configuration.
 */
namespace steer2D {}

/**
 * \namespace convergence
 *
 * \brief Swarm convergence measures and calculators: entropy, angular order,
 * velocity congruence, interactivity, etc., and their configuration.
 */
namespace convergence {
namespace config {}
}

/**
 * \namespace metrics
 *
 * \brief Reusable aggregators (groups of metric collectors) for the different
 * types of metrics that can be collected from COSM and their configuration.
 */
namespace metrics {
namespace config {}
} /* namespace metrics */

/**
 * \namespace repr
 *
 * \brief Representation of STUFF within simulation, that is not strictly a data
 *        structure, mathematical construct, etc., but that is used to model
 *        things in the real world.
 */
namespace repr {

/**
 * \namespace config
 *
 * \brief Configuration for some repsentation classes.
 */
namespace config {}

/**
 * \namespace operations
 *
 * \brief Common operations (action classes) on some representation types.
 */

namespace operations {}
} /* namespace repr */

/**
 * \namespace controller
 *
 * \brief Base classes for robot controllers that actuate in 2D, and that sense
 * in 2D or 3D, common operations on them (pickup a block, drop a block, etc.),
 * and metric interfaces
 */
namespace controller {
namespace operations {}
namespace metrics {}
} /* namespace controller */

/**
 * \namespace robots
 *
 * \brief Adaptor layer containing specializations/extensions of controllers,
 *        subsystems, etc., that are specific to a particular robot model. That
 *        is, each robot model COSM supports/has been tested with has a
 *        directory/namespace inside here.
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

/**
 * \namespace ds
 *
 * \brief Data structures used throughout COSM, and their associated
 * configuration and some common operations.
 */
namespace ds {
namespace config {}
namespace operations {}
} /* namespace ds */

/**
 * \namespace pal
 *
 * \brief Platform Abstraction Layer. Compile time switching between different
 * swarm manager platforms (whatever is running above/in addition to robot
 * controllers and collects metrics from them, processes, events, etc.). Also
 * contains configuration, operations, and temporal variance adapter classes for
 * each platform.
 */
namespace pal {
namespace config {}
namespace operations {}
namespace tv {}
} /* namespace pal */

/**
 * \namespace ta
 *
 * \brief Task allocation machinery for different task allocation strategies:
 * greedy, stochastic choice, random, UCB, etc. Also contains a runtime task
 * executive which can use any include strategy, necessary configuration, data
 * structures, and some metrics which can be collected about task
 * allocation/from tasks as they run.
 */
namespace ta {
namespace metrics {}
} /* namespace ta */


/**
 * \namespace spatial
 *
 * \brief Classes relating to spatial aspects of the arena in which swarms
 * operate: FSMs for acquiring spatial goal locations, spatial conflict
 * checkers, metrics, and different spatial strategies for exploration,
 * collision avoidance, etc.
 */
namespace spatial {
namespace fsm {}
namespace metrics {}

namespace strategy {
namespace metrics {}
namespace nest_acq {}
} /* namespace strategy */

} /* namespace spatial */

/**
 * \namespace foraging
 *
 * \brief Reusable machinery for foraging applications.
 *
 * - Block distriibution
 * - Representation of block clusters
 * - Data structures
 * - Oracles for object locations which can inject perfect knowledge of arena
 *   state into robot controllers.
 * - FSMs.
 */
namespace foraging {
namespace config {}
namespace block_dist {}
namespace repr {}
namespace ds {}
namespace oracle {}
namespace metrics {}
namespace fsm {}
} /* namespace foraging */

/**
 * \namespace arena
 *
 * \brief The arena map which swarm managers from the PAL use as a data store
 * for simulation state. Contains arena internal data structures, operations on
 * the arena (e.g., block pickup/drop), representation of arena internal objects
 * such as caches, and metric interfaces.
 */
namespace arena {
namespace operations {}
namespace ds {}
namespace repr {}
namespace metrics {}
namespace config {}

} /* namespace arena */

/**
 * \namespace oracle
 *
 * \brief Oracles for injecting  perfect knowledge of simulation state into
 *  robot controllers.
 */
namespace oracle {
namespace config {}
} /* namespace oracle */

/**
 * \namespace interactors
 *
 * \brief Base classes for some robot-arena interactions such as block pickup,
 * block drop, task abort, etc.
 */
namespace interactors {}
} /* namespace cosm */

namespace csubsystem = cosm::subsystem;
namespace csperception = csubsystem::perception;
namespace cspconfig = csperception::config;

namespace cinteractors = cosm::interactors;
namespace ccontroller = cosm::controller;
namespace ccops = ccontroller::operations;
namespace ccmetrics = ccontroller::metrics;

namespace cforaging = cosm::foraging;
namespace cfrepr = cforaging::repr;
namespace cforacle = cforaging::oracle;
namespace cfds = cforaging::ds;
namespace cfconfig = cforaging::config;
namespace cfbd = cforaging::block_dist;
namespace cfmetrics = cforaging::metrics;
namespace cffsm = cforaging::fsm;

namespace cspatial = cosm::spatial;
namespace csfsm = cspatial::fsm;
namespace csmetrics = cspatial::metrics;
namespace csstrategy = cspatial::strategy;
namespace cssmetrics = csstrategy::metrics;
namespace cssnest_acq = csstrategy::nest_acq;

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
