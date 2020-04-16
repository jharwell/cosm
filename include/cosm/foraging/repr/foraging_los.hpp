/**
 * \file foraging_los.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_FORAGING_REPR_FORAGING_LOS_HPP_
#define INCLUDE_COSM_FORAGING_REPR_FORAGING_LOS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/multi_array.hpp>
#include <list>
#include <utility>

#include "rcppsw/ds/base_grid2D.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"

#include "cosm/arena/ds/cache_vector.hpp"
#include "cosm/ds/entity_vector.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ds {
class cell2D;
} /* namespace cosm::ds */

NS_START(cosm, foraging, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class foraging_los
 * \ingroup foraging repr
 *
 * \brief A repr of the robot's current line-of-sight in 2D. The robot is
 * only able to update its internal state based on the information present in
 * the per-timestep updates to this object.
 *
 * The LOS for a robot is always square UNLESS the robot is near the edge of
 * the arena, and a square grid would result in out-of-bounds array accesses. In
 * that case, a truncated LOS is created.
 *
 * All coordinates within a LOS are relative to the LOS itself (not its location
 * within the arena). The origin is in the lower left corner of the LOS.
 *
 * The line of sight itself is meant to be a read-only view of part of the
 * arena, but it also exposes non-const access to the blocks and caches within
 * that part of the arena by necessity for event processing.
 */
class foraging_los final : public rer::client<foraging_los> {
 public:
  using grid_view = rds::base_grid2D<cds::cell2D>::grid_view;
  using const_grid_view = rds::base_grid2D<cds::cell2D>::const_grid_view;

  foraging_los(const const_grid_view& c_view, const rmath::vector2z& center)
      : ER_CLIENT_INIT("cosm.foraging.repr.foraging_los"),
        mc_center(center),
        mc_view(c_view) {}

  /**
   * \brief Get the list of blocks currently in the LOS.
   *
   * This has to return a \ref cds::entity_vector, because we cannot generically
   * know if there are only 2D blocks, only 3D blocks, or a mix, currently in
   * the arena, so we have to do the generic thing. As long as a derived project
   * uses exclusively 2D or 3D blocks, and doesn't mix them, then this is a zero
   * cost abstraction, because it can be casted away with static_cast at compile
   * time.
   */
  cds::entity_vector blocks(void) const;
  cads::bcache_vectorno caches(void) const;

  /**
   * \brief Get the size of the X dimension for a LOS.
   *
   * \return The X dimension.
   */
  size_t xsize(void) const { return mc_view.shape()[0]; }

  rmath::vector2z abs_ll(void) const RCSW_PURE;
  rmath::vector2z abs_lr(void) const RCSW_PURE;
  rmath::vector2z abs_ul(void) const RCSW_PURE;
  rmath::vector2z abs_ur(void) const RCSW_PURE;

  /**
   * \brief Get the size of the Y dimension for a LOS.
   *
   * \return The Y dimension.
   */
  grid_view::size_type ysize(void) const { return mc_view.shape()[1]; }

  /**
   * \brief Determine if the *ABSOLUTE* arena location is contained in the LOS.
   */
  bool contains_loc(const rmath::vector2z& loc) const RCSW_PURE;

  /**
   * \brief Get the # elements in a LOS.
   *
   * \return # elements.
   */
  grid_view::size_type size(void) const { return mc_view.num_elements(); }

  /**
   * \brief Get the cell associated with a particular grid location within the
   * LOS. Asserts that both coordinates are within the bounds of the grid
   * underlying the LOS.
   *
   * \param i The RELATIVE X coord within the LOS.
   * \param j The RELATIVE Y coord within the LOS.
   *
   * \return A reference to the cell.
   */
  const cds::cell2D& cell(uint i, uint j) const RCSW_PURE;

  /**
   * \brief Get the coordinates for the center of the LOS.
   *
   * \return The center coordinates (discrete version).
   */
  const rmath::vector2z& center(void) const { return mc_center; }

 private:
  /* clang-format off */
  const rmath::vector2z mc_center;
  const const_grid_view mc_view;
  /* clang-format on */
};

NS_END(repr, foraging, cosm);

#endif /* INCLUDE_COSM_FORAGING_REPR_FORAGING_LOS_HPP_ */
