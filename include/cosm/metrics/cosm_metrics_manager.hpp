/**
 * \file cosm_metrics_manager.hpp
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

#ifndef INCLUDE_COSM_METRICS_COSM_METRICS_MANAGER_HPP_
#define INCLUDE_COSM_METRICS_COSM_METRICS_MANAGER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/math/vector3.hpp"

#include "rcppsw/metrics/config/metrics_config.hpp"
#include "rcppsw/metrics/base_metrics_manager.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {
class base_block3D;
} /* namespace cosm::repr */

namespace cosm::arena {
class base_arena_map;
} /* namespace cosm::arena */

namespace cosm::controller {
class base_controller2D;
class base_controllerQ3D;
} /* namespace cosm::controller */

NS_START(cosm, metrics);
namespace fs = std::filesystem;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cosm_metrics_manager
 * \ingroup metrics
 *
 * \brief Manager class for handling all of the metrics which can be generated
 * by COSM.
 */
class cosm_metrics_manager : public rmetrics::base_metrics_manager,
                             public rer::client<cosm_metrics_manager> {
 public:
  cosm_metrics_manager(const rmconfig::metrics_config* mconfig,
                       const fs::path& output_root);
  ~cosm_metrics_manager(void) override = default;

  /**
   * \brief Collect metrics from the arena. Currently this includes:
   *
   * - \c blocks::motion
   * - \c blocks::distributor
   * - \c blocks::clusters (from each block cluster in the arena)
   */
  void collect_from_arena(const carena::base_arena_map* map);

  /**
   * \brief Collect metrics from a 3D block right before it is dropped in the
   * nest. Currently this includes:
   *
   * - \c blocks::transportee
   */
  void collect_from_block(const crepr::base_block3D* block);

  /**
   * \brief Collect metrics from 2D controllers. Currently this includes:
   *
   * - \c spatial::dist::pos2D
   * - \c spatial::movement
   * - \c spatial::interference::counts
   * - \c spatial::interference::locs2D
   */
  void collect_from_controller(const controller::base_controller2D* controller);

  /**
   * \brief Collect metrics from Q3D controllers. Currently this includes:
   *
   * - \c spatial::dist::pos3D
   * - \c spatial::movement
   * - \c spatial::interference::counts
   * - \c spatial::interference::locs3D
   */
  void collect_from_controller(const controller::base_controllerQ3D* controller);

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

NS_END(metrics, cosm);

#endif /* INCLUDE_COSM_METRICS_COSM_METRICS_MANAGER_HPP_ */
