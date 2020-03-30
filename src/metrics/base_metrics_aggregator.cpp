/**
 * \file base_metrics_aggregator.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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
#include "cosm/metrics/base_metrics_aggregator.hpp"

#include <boost/mpl/for_each.hpp>

#include "rcppsw/mpl/typelist.hpp"

#include "cosm/convergence/convergence_calculator.hpp"
#include "cosm/convergence/metrics/convergence_metrics.hpp"
#include "cosm/convergence/metrics/convergence_metrics_collector.hpp"
#include "cosm/fsm/metrics/collision_locs_metrics_collector.hpp"
#include "cosm/fsm/metrics/collision_metrics.hpp"
#include "cosm/fsm/metrics/collision_metrics_collector.hpp"
#include "cosm/fsm/metrics/current_explore_locs_metrics_collector.hpp"
#include "cosm/fsm/metrics/current_vector_locs_metrics_collector.hpp"
#include "cosm/fsm/metrics/goal_acq_locs_metrics_collector.hpp"
#include "cosm/fsm/metrics/goal_acq_metrics.hpp"
#include "cosm/fsm/metrics/goal_acq_metrics_collector.hpp"
#include "cosm/fsm/metrics/movement_metrics.hpp"
#include "cosm/fsm/metrics/movement_metrics_collector.hpp"
#include "cosm/metrics/blocks/transport_metrics_collector.hpp"
#include "cosm/metrics/spatial/dist2D_metrics.hpp"
#include "cosm/metrics/spatial/dist2D_pos_metrics_collector.hpp"
#include "cosm/metrics/spatial/dist3D_metrics.hpp"
#include "cosm/metrics/spatial/dist3D_pos_metrics_collector.hpp"
#include "cosm/tv/metrics/population_dynamics_metrics.hpp"
#include "cosm/tv/metrics/population_dynamics_metrics_collector.hpp"
#include "cosm/metrics/collector_registerer.hpp"
#include "cosm/controller/base_controller2D.hpp"
#include "cosm/controller/base_controllerQ3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, metrics, detail);

using collector_typelist = rmpl::typelist<
    rmpl::identity<cfsm::metrics::movement_metrics_collector>,
    rmpl::identity<cfsm::metrics::collision_metrics_collector>,
    rmpl::identity<cfsm::metrics::collision_locs_metrics_collector>,
    rmpl::identity<cfsm::metrics::goal_acq_metrics_collector>,
    rmpl::identity<cfsm::metrics::goal_acq_locs_metrics_collector>,
    rmpl::identity<cfsm::metrics::current_explore_locs_metrics_collector>,
    rmpl::identity<cfsm::metrics::current_vector_locs_metrics_collector>,
    rmpl::identity<cmetrics::blocks::transport_metrics_collector>,
    rmpl::identity<cmetrics::spatial::dist2D_pos_metrics_collector>,
    rmpl::identity<cconvergence::metrics::convergence_metrics_collector>,
    rmpl::identity<ctvmetrics::population_dynamics_metrics_collector> >;

NS_END(detail);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
base_metrics_aggregator::base_metrics_aggregator(
    const cmconfig::metrics_config* const mconfig,
    const cdconfig::grid_config* const gconfig,
    const std::string& output_root)
    : ER_CLIENT_INIT("cosm.metrics.base_aggregator"),
      m_metrics_path(fs::path(output_root) / mconfig->output_dir) {
  if (!fs::exists(m_metrics_path)) {
    fs::create_directories(m_metrics_path);
  } else {
    ER_WARN("Output metrics path '%s' already exists", m_metrics_path.c_str());
  }
  collector_registerer::creatable_set creatable_set = {
      {typeid(cfsm::metrics::movement_metrics_collector),
       "fsm_movement",
       "fsm::movement",
       rmetrics::output_mode::ekAPPEND},
      {typeid(cfsm::metrics::collision_locs_metrics_collector),
       "fsm_collision_locs",
       "fsm::collision_locs",
       rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE},
      {typeid(cfsm::metrics::collision_metrics_collector),
       "fsm_collision_counts",
       "fsm::collision_counts",
       rmetrics::output_mode::ekAPPEND},
      {typeid(cfsm::metrics::goal_acq_metrics_collector),
       "block_acq_counts",
       "blocks::acq_counts",
       rmetrics::output_mode::ekAPPEND},
      {typeid(cfsm::metrics::goal_acq_locs_metrics_collector),
       "block_acq_locs",
       "blocks::acq_locs",
       rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE},
      {typeid(cfsm::metrics::current_explore_locs_metrics_collector),
       "block_acq_explore_locs",
       "blocks::acq_explore_locs",
       rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE},
      {typeid(cfsm::metrics::current_vector_locs_metrics_collector),
       "block_acq_vector_locs",
       "blocks::acq_vector_locs",
       rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE},
      {typeid(cmetrics::blocks::transport_metrics_collector),
       "block_transport",
       "blocks::transport",
       rmetrics::output_mode::ekAPPEND},
      {typeid(cmetrics::spatial::dist2D_pos_metrics_collector),
       "swarm_dist2D_pos",
       "swarm::spatial_dist2D::pos",
       rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE},
      {typeid(cmetrics::spatial::dist3D_pos_metrics_collector),
       "swarm_dist3D_pos",
       "swarm::spatial_dist3D::pos",
       rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE},
      {typeid(cconvergence::metrics::convergence_metrics_collector),
       "swarm_convergence",
       "swarm::convergence",
       rmetrics::output_mode::ekAPPEND},
      {typeid(ctvmetrics::population_dynamics_metrics_collector),
       "tv_population",
       "tv::population",
       rmetrics::output_mode::ekAPPEND}};

  collector_registerer registerer(mconfig, gconfig, creatable_set, this);
  boost::mpl::for_each<detail::collector_typelist>(registerer);
  reset_all();
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void base_metrics_aggregator::collect_from_block(
    const crepr::base_block2D* const block) {
  collect("blocks::transport", *block->md());
} /* collect_from_block() */

void base_metrics_aggregator::collect_from_block(
    const crepr::base_block3D* const block) {
  collect("blocks::transport", *block->md());
} /* collect_from_block() */

void base_metrics_aggregator::collect_from_controller(
    const controller::base_controller2D* const controller) {
  collect("swarm::spatial_dist2D::pos", *controller);
} /* collect_from_controller() */

void base_metrics_aggregator::collect_from_controller(
    const controller::base_controllerQ3D* const controller) {
  collect("swarm::spatial_dist3D::pos", *controller);
} /* collect_from_controller() */

NS_END(metrics, cosm);
