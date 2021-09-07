/**
 * \file base_grid_view_entity.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_REPR_BASE_GRID_VIEW_ENTITY_HPP_
#define INCLUDE_COSM_REPR_BASE_GRID_VIEW_ENTITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/types/discretize_ratio.hpp"
#include "rcppsw/patterns/decorator/decorator.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_grid_view_entity
 * \ingroup repr
 *
 * \brief Representation of an \ref arena_grid::grid_view or \ref
 * arena_grid::const_grid_view object (or something similar) as an entity in the
 * arena. This representation makes it much easier to pass abstract, mult-cell
 * objects that are not entities per-se as entities for various operations such
 * as block distribution in foraging.
 *
 * It has the following characteristics:
 *
 * - Spans multiple cells in the arena.
 * - Does not "exist" in the sense that it is not detectable by robots.
 * - Has no concept of movability/immovability (again, it is abstract).
 */
template <typename TGridType, typename TGridViewType>
class base_grid_view_entity : public rpdecorator::decorator<TGridViewType> {
 public:
  using grid_type = TGridType;
  using grid_view_type = TGridViewType;
  using cell_type = typename grid_type::value_type;
  using coord_type = typename grid_type::coord_type;

  base_grid_view_entity(const grid_view_type& view,
                        const rtypes::discretize_ratio& resolution)
      : rpdecorator::decorator<grid_view_type>(view),
        mc_resolution(resolution) {}

  virtual ~base_grid_view_entity(void) = default;

  const rtypes::discretize_ratio& resolution(void) const {
    return mc_resolution;
  }

  /**
   * \brief Get the cell associated with a particular grid location within the
   * view. Asserts that both coordinates are within the bounds of the grid
   * underlying the view.
   *
   * \param c The RELATIVE coord within the view.
   *
   * \return A reference to the cell.
   */
  virtual const cell_type& access(const coord_type& c) const = 0;

  /**
   * \brief Determine if the specified ABSOLUTE coordinates lie within the view
   * entity (i.e., the coordinates are specified relative to the origin of the
   * parent grid this entity is built from.)
   */
  virtual bool contains_abs(const coord_type& cell) const = 0;

  /**
   * \brief Determine if the specified RELATIVE coordinates lie within the view
   * entity (i.e., the coordinates are specified relative to the origin of the
   * view entity, which is not necessarily the same as the origin of the parent
   * grid).
   */
  virtual bool contains_rel(const coord_type& cell) const = 0;

 protected:
  const grid_view_type& view(void) const { return decoratee(); }

 private:
  using rpdecorator::decorator<grid_view_type>::decoratee;

  /* clang-format off */
  const rtypes::discretize_ratio mc_resolution;
  /* clang-format on */
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_BASE_GRID_VIEW_ENTITY_HPP_ */
