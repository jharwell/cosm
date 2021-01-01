/**
 * \file block_motion_handler.hpp
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

#ifndef INCLUDE_COSM_FORAGING_BLOCK_MOTION_HANDLER_HPP_
#define INCLUDE_COSM_FORAGING_BLOCK_MOTION_HANDLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/optional.hpp>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/rng.hpp"
#include "rcppsw/math/vector2.hpp"

#include "cosm/cosm.hpp"
#include "cosm/foraging/config/block_motion_config.hpp"
#include "cosm/foraging/metrics/block_motion_metrics.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::repr {
class base_block3D;
} /* namespace cosm::repr */

namespace cosm::arena {
class base_arena_map;
} /* namespace cosm::arena */

NS_START(cosm, foraging);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_motion_handler
 * \ingroup foraging
 *
 * \brief Moves free blocks in the area according to the configured policy.
 */
class block_motion_handler : public rer::client<block_motion_handler>,
                             public cfmetrics::block_motion_metrics {
 public:
  static constexpr const char kPolicyNull[] = "Null";
  static constexpr const char kPolicyRandomWalk[] = "random_walk";

  block_motion_handler(const config::block_motion_config* config,
                       rmath::rng* rng);

  /* Not move/copy constructable/assignable by default */
  block_motion_handler(const block_motion_handler&) = delete;
  const block_motion_handler& operator=(const block_motion_handler&) = delete;
  block_motion_handler(block_motion_handler&&) = delete;
  block_motion_handler& operator=(block_motion_handler&&) = delete;

  /* motion metrics */
  size_t n_moved(void) const override { return m_n_moved; }

  /**
   * \brief Check each free block in the arena to see if it should be moved, and
   * move blocks as needed.
   *
   * \return The # of blocks moved.
   */
  size_t move_blocks(carena::base_arena_map* map);

 private:
  /**
   * \brief The # of tries to move a free block to an adjacent square. We
   * have 8 total squares that a given block can be moved to, and we choose
   * between them uniformly.
   */
  static constexpr const size_t kMAX_TRIES = 32;

  void random_walk(carena::base_arena_map* map);

  boost::optional<rmath::vector2z>
  free_adjacent_coord(const crepr::base_block3D* block,
                      const carena::base_arena_map* map);

  /* clang-format off */
  const config::block_motion_config mc_config;

  size_t                            m_n_moved{0};
  rmath::rng*                       m_rng;
  /* clang-format on */
};

NS_END(foraging, cosm);

#endif /* INCLUDE_COSM_FORAGING_BLOCK_MOTION_HANDLER_HPP_ */
