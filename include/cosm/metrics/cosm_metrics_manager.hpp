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
   * \ref foraging::block_dist::metrics::distributor_metrics
   * \ref foraging::metrics::block_motion_metrics
   */
  void collect_from_arena(const carena::base_arena_map* map);

  /**
   * \brief Collect metrics from a 3D block right before it is dropped in the
   * nest.
   */
  void collect_from_block(const crepr::base_block3D* block);

  /**
   * \brief Collect metrics from 2D controllers. Currently this includes:
   *
   * - \ref spatial::metrics::dist2D_metrics
   * - \ref spatial::metrics::movement_metrics
   * - \ref spatial::metrics::collision_metrics
   */
  void collect_from_controller(const controller::base_controller2D* controller);

  /**
   * \brief Collect metrics from Q3D controllers. Currently this includes:
   *
   * - \ref spatial::metrics::dist3D_metrics
   * - \ref spatial::metrics::movement_metrics
   * - \ref spatial::metrics::collision_metrics
   */
  void collect_from_controller(const controller::base_controllerQ3D* controller);

 protected:
  /**
   * \brief Register metrics collectors that require the 2D arena dimensions to
   * construct.
   *
   * - fsm::interference_locs2D
   * - blocks::acq_locs2D
   * - blocks::acq_explore_locs2D
   * - blocks::acq_vector_locs2D
   * - swarm::spatial_dist2D::pos
   */
  void register_with_arena_dims2D(const rmconfig::metrics_config* mconfig,
                                  const rmath::vector2z& dims);

  /**
   * \brief Register metrics collectors that require the 3D arena dimensions to
   * construct.
   *
   * - fsm::interference_locs3D
   * - swarm::spatial_dist3D::pos
   */
  void register_with_arena_dims3D(const rmconfig::metrics_config* mconfig,
                                  const rmath::vector3z& dims);

  /**
   * \brief Register metrics collectors that require the # of block clusters in
   * the arena.
   *
   * - blocks::clusters
   */
  void register_with_n_block_clusters(const rmconfig::metrics_config* mconfig,
                                      size_t n_block_clusters);

 private:
  /**
   * \brief Register metrics collectors that do not require extra arguments.
   *
   * - spatial::movement
   * - fsm::interference_counts
   * - blocks::acq_counts
   * - blocks::transportee
   * - blocks::transport
   * - blocks::distributor
   * - blocks::motion
   * - swarm::convergence
   * - tv::population
   */
  void register_standard(const rmconfig::metrics_config* mconfig);
};

NS_END(metrics, cosm);

#endif /* INCLUDE_COSM_METRICS_COSM_METRICS_MANAGER_HPP_ */
