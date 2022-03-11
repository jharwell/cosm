/**
 * \file fs_output_manager.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/math/vector3.hpp"

#include "rcppsw/metrics/config/metrics_config.hpp"
#include "rcppsw/metrics/fs_output_manager.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {
class sim_block3D;
} /* namespace cosm::repr */

namespace cosm::arena {
class base_arena_map;
} /* namespace cosm::arena */

namespace cosm::controller {
class base_controller2D;
class base_controllerQ3D;
} /* namespace cosm::controller */

NS_START(cosm, argos, metrics);
namespace fs = std::filesystem;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class fs_output_manager
 * \ingroup argos metrics
 *
 * \brief Manager class for handling all of the metrics which can be generated
 * by COSM on the ARGoS platform.
 */
class fs_output_manager : public rmetrics::fs_output_manager,
                                        public rer::client<fs_output_manager> {
 public:
  fs_output_manager(const rmconfig::metrics_config* mconfig,
                       const fs::path& output_root);
  ~fs_output_manager(void) override = default;

  /**
   * \brief Collect metrics from the arena. Currently this includes:
   *
   * - \ref specs::blocks::kMotion
   * - \ref specs::blocks::kDistributor
   * - \ref specs::blocks::kClusters (from each block cluster in the arena)
   */
  void collect_from_arena(const carena::base_arena_map* map);

  /**
   * \brief Collect metrics from a 3D block right before it is dropped in the
   * nest. Currently this includes:
   *
   * - \ref specs::blocks::kTransportee
   */
  void collect_from_block(const crepr::sim_block3D* block);

  /**
   * \brief Collect metrics from 2D controllers. Currently this includes:
   *
   * - \ref specs::spatial::kDistPosition2D
   * - \ref specs::spatial::kMovement
   * - \ref specs::spatial::kInterferenceCounts
   * - \ref specs::spatial::kInterferenceLocs2D
   */
  void collect_from_controller(const ccontroller::base_controller2D* controller);

  /**
   * \brief Collect metrics from Q3D controllers. Currently this includes:
   *
   * - \ref spatial::DistPosition3D
   * - \ref spatial::kMovement
   * - \ref spatial::kInterferenceCounts
   * - \ref spatial::InterferenceLocs3D
   */
  void collect_from_controller(const ccontroller::base_controllerQ3D* controller);

 protected:
  /**
   * \brief Register metrics collectors that require the 2D arena dimensions to
   * construct.
   *
   * - \c spatial::dist::pos2D -> \ref csmetrics::dist2D_metrics
   * - \c spatial::interference::locs2D -> \ref csmetrics::interference_metrics
   * - \c blocks::acq::explore_locs2D -> \ref csmetrics::goal_acq_metrics
   * - \c blocks::acq::locs2D -> \ref csmetrics::goal_acq_metrics
   * - \c blocks::acq::vector_locs2D \ref csmetrics::goal_acq_metrics
   */
  void register_with_arena_dims2D(const rmconfig::metrics_config* mconfig,
                                  const rmath::vector2z& dims);

  /**
   * \brief Register metrics collectors that require the 3D arena dimensions to
   * construct.
   *
   * - \c spatial::dist::pos3D -> \ref csmetrics::dist3D_metrics
   * - \c spatial::interference::locs3D -> \ref csmetrics::interference_metrics
   * - \c blocks::acq::explore_locs3D -> \ref csmetrics::goal_acq_metrics
   * - \c blocks::acq::vector_locs3D -> \ref csmetrics::goal_acq_metrics
   */
  void register_with_arena_dims3D(const rmconfig::metrics_config* mconfig,
                                  const rmath::vector3z& dims);

  /**
   * \brief Register metrics collectors that require the # of block clusters in
   * the arena.
   *
   * - \c blocks::clusters -> \ref cfmetrics::block_cluster_metrics
   */
  void register_with_n_block_clusters(const rmconfig::metrics_config* mconfig,
                                      size_t n_block_clusters);

 private:
  /**
   * \brief Register metrics collectors that do not require extra arguments.
   *
   * - \c swarm::convergence -> \ref cconvergence::metrics::convergence_metrics
   * - \c spatial::movement -> \ref csmetrics::movement_metrics
   * - \c spatial::interference::counts -> \ref csmetrics::interference_metrics
   * - \c spatial::nest_zone -> \ref csmetrics::nest_zone_metrics
   * - \c blocks::distributor -> \ref cfbd::metrics::distributor_metrics
   * - \c blocks::motion -> \ref cfmetrics::block_motion_metrics
   * - \c blocks::transporter -> \ref cfsm::metrics::block_transporter_metrics
   * - \c blocks::transportee -> \ref cfmetrics::block_transportee_metrics
   * - \c blocks::acq_counts -> \ref csmetrics::goal_acq_metrics
   * - \c strategy::nest_acq -> \ref cssmetrics::nest_acq_metrics
   * - \c tv::population -> \ref ctvmetrics::population_dynamics_metrics
   */
  void register_standard(const rmconfig::metrics_config* mconfig);
};

NS_END(metrics, argos, cosm);
