/**
 * \file cosm.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/rcppsw.hpp"

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
 * configuration. Used to modify swarm behavior over time (e.g., forcing agents
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

namespace argos {
namespace operations {}
namespace tv {}
/**
 * \namespace vis vis
 * \brief Qt visualization bindings.
 */
namespace vis {
namespace config {}
} /* namespace vis */
namespace interactors {}
} /* namespace argos */

namespace ros {
namespace metrics {}
namespace interactors {}
namespace config {}
namespace foraging {
namespace metrics {}
} /* namespace foraging */
namespace fsm {
namespace metrics {}
} /* namespace fsm */

namespace spatial {
namespace metrics {}
} /* namespace spatial */

} /* namespace ros */

/**
 * \namespace subsystem
 *
 * \brief Classes for agent subsystem such as sensing, perception, and
 * actuation, and their configuration. Not tied to a specific agent type.
 */
namespace subsystem {
namespace config {}
namespace perception {
namespace rlos {}
namespace config {}
} /* namespace perception */
} /* namespace subsystem */

/**
 * \namespace hal
 *
 * \brief Hardware Abstraction Layer (HAL) for compile time switching between
 * different implementations of agent sensors and actuators.
 *
 * Useful to:
 *
 * - Eliminate run-time penalties when de-muxing
 *
 * - Present a uniform interface to higher level modules, such as \ref subystem
 *   classes, regardless of which platform/agent type COSM is built for.
 */
namespace hal {

namespace argos {
namespace sensors {}
namespace actuators {}
namespace config {}
} /* namespace argos */

namespace ros {
namespace sensors {}
namespace actuators {}
namespace config {}
} /* namespace ros */

namespace subsystem {
namespace config {}
} /* namespace subsystem */

namespace sensors {}
namespace actuators {}
} /* namespace hal */

/**
 * \namespace kin
 *
 * \brief Classes for representing and tracking agent kinematics for agents
 * for which doing so is meaningful.
 */
namespace kin {
namespace metrics {}

} /* namespace kin */

/**
 * \namespace kin2D
 *
 * \brief Differential drive classes for wheeled agents moving in 2D, and their
 * configuration.
 */
namespace kin2D {}

/**
 * \namespace apf2D
 *
 * \brief Artificial Potential Fields (APFs), inspired from Arkin's original
 * 1987 paper.
 *
 * Arrival, avoidance, wander, etc., and their configuration. Generally 2D.
 */
namespace apf2D {}

/**
 * \namespace convergence
 *
 * \brief Swarm convergence measures and calculators: entropy, angular order,
 * velocity congruence, interactivity, etc., and their configuration.
 */
namespace convergence {
namespace metrics {
}
namespace config {}
}

/**
 * \namespace metrics
 *
 * \brief Reusable aggregators (groups of metric collectors) for the different
 * types of metrics that can be collected from COSM and their configuration.
 */
namespace metrics {
namespace specs {}
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
 * \brief Base classes for agent controllers that sense in 3D/actuate in 2D,
 * sense/actuate in 3D, common operations on them (pickup a block, drop a block,
 * etc.), and metric interfaces.
 */
namespace controller {
namespace operations {}
namespace metrics {}
} /* namespace controller */

/**
 * \namespace fsm
 * \brief Reusable FSMs not specific to any particular application within MAS.
 */
namespace fsm {}

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
 * \brief Platform Abstraction Layer (PAL)for compile time switching between different
 * swarm manager platforms (whatever is running above/in addition to agent
 * controllers and collects metrics from them, processes, events, etc.).
 *
 * Also contains configuration, operations, and temporal variance adapter
 * classes for each platform.
 */
namespace pal {
namespace controller {}
namespace metrics {}
namespace config {
namespace xml {}
}
namespace argos {
namespace controller {}
} /* namespace argos */
namespace ros {
namespace controller {}

}
} /* namespace pal */

/**
 * \namespace ta
 *
 * \brief Task allocation machinery for different task allocation strategies:
 * greedy, stochastic choice, random, UCB, etc.
 *
 * Also contains a runtime task executive which can use any include strategy,
 * necessary configuration, data structures, and some metrics which can be
 * collected about task allocation/from tasks as they run.
 */
namespace ta {
namespace metrics {}
} /* namespace ta */


/**
 * \namespace spatial
 *
 * \brief Classes relating to spatial aspects of the arena in which swarms
 * operate..
 *
 * FSMs for acquiring spatial goal locations, spatial conflict
 * checkers, metrics, and different spatial strategies for exploration,
 * collision avoidance, etc.
 */
namespace spatial {
namespace fsm {}
namespace metrics {}

namespace strategy {
namespace metrics {}
namespace nest {
namespace acq {}
namespace exit {}
}

namespace blocks {
namespace drop {}
} /* namespace blocks */

namespace explore {}

namespace flocking {
} /* namespace flocking */

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
 *   state into agent controllers.
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
 * \namespace flocking
 *
 * \brief Reusable machinery for flocking applications.
 */
namespace flocking {}

/**
 * \namespace arena
 *
 * \brief The arena map which swarm managers from the PAL use as a data store
 * for experimental run state.
 *
 * Contains arena internal data structures, operations on the arena (e.g., block
 * pickup/drop), representation of arena internal objects such as caches, and
 * metric interfaces.
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
 *  agent controllers.
 */
namespace oracle {
namespace config {}
} /* namespace oracle */

/**
 * \namespace interactors
 *
 * \brief Base classes for some agent-arena interactions such as block pickup,
 * block drop, task abort, etc.
 */
namespace interactors {}

/**
 * \namespace nav
 *
 * Classes related to navigation generally which are not specific to the method
 * of navigation (e.g., APF).
 */
namespace nav {

namespace config {}

} /* namespace nav */

} /* namespace cosm */

namespace csubsystem = cosm::subsystem;
namespace csconfig = csubsystem::config;
namespace csperception = csubsystem::perception;
namespace cspconfig = csperception::config;
namespace csprlos = csperception::rlos;

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
namespace cssnest = csstrategy::nest;
namespace cssblocks = csstrategy::blocks;
namespace cssexplore = csstrategy::explore;
namespace cssflocking = csstrategy::flocking;

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
namespace cmspecs = cmetrics::specs;
namespace cmconfig = cmetrics::config;

namespace cds = cosm::ds;
namespace cdconfig = cds::config;
namespace cdops = cds::operations;

namespace cpal = cosm::pal;
namespace cpconfig = cpal::config;
namespace cpmetrics = cpal::metrics;
namespace cpcxml = cpconfig::xml;
namespace cpargos = cpal::argos;
namespace cpros = cpal::ros;
namespace cpcontroller = cpal::controller;

namespace cargos = cosm::argos;
namespace catv = cargos::tv;
namespace cavis = cargos::vis;
namespace cainteractors = cargos::interactors;

namespace cros = cosm::ros;
namespace crmetrics = cros::metrics;
namespace crinteractors = cros::interactors;
namespace crforaging = cros::foraging;
namespace crfmetrics = crforaging::metrics;
namespace crfsm = cros::fsm;
namespace crspatial = cros::spatial;
namespace crsmetrics = crspatial::metrics;

namespace coracle = cosm::oracle;
namespace coconfig = coracle::config;

namespace crepr = cosm::repr;
namespace crops = crepr::operations;

namespace chal = cosm::hal;
namespace chsensors = chal::sensors;
namespace chactuators = chal::actuators;

namespace chros = chal::ros;
namespace chrsensors = chros::sensors;
namespace chractuators = chros::actuators;
namespace chrconfig = chros::config;

namespace chargos = chal::argos;
namespace chasensors = chargos::sensors;
namespace chaactuators = chargos::actuators;
namespace chaconfig = chargos::config;
namespace chsubsystem = chal::subsystem;
namespace chsconfig = chsubsystem::config;

namespace crconfig = crepr::config;

namespace cfsm = cosm::fsm;

namespace ckin = cosm::kin;
namespace ckmetrics = ckin::metrics;

namespace ckin2D = cosm::kin2D;
namespace capf2D = cosm::apf2D;

namespace cflocking = cosm::flocking;

namespace cnav = cosm::nav;
namespace cnconfig = cnav::config;
