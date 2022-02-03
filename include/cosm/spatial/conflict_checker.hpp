/**
 * \file conflict_checker.hpp
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

#ifndef INCLUDE_COSM_SPATIAL_CONFLICT_CHECKER_HPP_
#define INCLUDE_COSM_SPATIAL_CONFLICT_CHECKER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <functional>
#include <boost/optional.hpp>

#include "rcppsw/math/vector2.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::arena {
class base_arena_map;
class caching_arena_map;
} /* namespace cosm::arena */

namespace cosm::repr {
class sim_block3D;
class entity2D;
class entity3D;
} /* namespace cosm::repr */

NS_START(cosm, spatial);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class conflict_checker
 * \ingroup spatial
 *
 * \brief Checker for various kinds of spatial conflicts in the arena. Prior to
 * creating this class, the checking functions were scattered all over in
 * various \a utils:: namespaces. Much better to have them all centralized.
 */
class conflict_checker {
 public:
  struct status {
    bool x{ false };
    bool y{ false };

    status& operator|=(const status& other) {
      x |= other.x;
      y |= other.y;
      return *this;
    }
  };
  using map_cb_type = std::function<status(const crepr::sim_block3D* const block,
                                           const rmath::vector2d& loc)>;

  /* Not move/copy constructable/assignable by default */
  conflict_checker(const conflict_checker&) = delete;
  const conflict_checker& operator=(const conflict_checker&) = delete;
  conflict_checker(conflict_checker&&) = delete;
  conflict_checker& operator=(conflict_checker&&) = delete;

  /**
   * \brief Check for conflict if the specified block is placed at the specified
   * location in the arena.
   *
   * The following are checked:
   *
   * - Within arena distributable area (not near boundaries, which can causing
   *   problems with robot block acquisition).
   * - Overlap with all other blocks.
   * - Overlap with all nests.
   */
  static status placement2D(const carena::base_arena_map* map,
                            const crepr::sim_block3D* block,
                            const rmath::vector2d& loc);

  /**
   * \brief Check for conflict if the specified block is placed at the specified
   * location in the arena.
   *
   * The following are checked:
   *
   * - Within arena distributable area (not near boundaries, which can causing
   *   problems with robot block acquisition).
   * - Overlap with all other blocks.
   * - Overlap with all nests.
   * - Overlap with all caches.
   */
  static status placement2D(const carena::caching_arena_map* map,
                            const crepr::sim_block3D* block,
                            const rmath::vector2d& loc);

  /**
   * \brief Determine if an entity anchored at \p ent1_anchor with dimensions
   * \p ent1_dims overlaps 2D entity \p ent2.
   */
  static status placement2D(const rmath::vector2d& ent1_anchor,
                            const rmath::vector2d& ent1_dims,
                            const crepr::entity2D* ent2);

  /**
   * \brief Determine if an entity anchored at \p ent1_anchor with dimensions
   * \p ent1_dims overlaps 3D entity \p ent2.
   */
  static status placement2D(const rmath::vector2d& ent1_anchor,
                            const rmath::vector2d& ent1_dims,
                            const crepr::entity3D* entity);
};

NS_END(spatial, cosm);

#endif /* INCLUDE_COSM_SPATIAL_CONFLICT_CHECKER_HPP_ */
