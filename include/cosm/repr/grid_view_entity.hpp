/**
 * \file grid_view_entity.hpp
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

#ifndef INCLUDE_COSM_REPR_GRID_VIEW_ENTITY_HPP_
#define INCLUDE_COSM_REPR_GRID_VIEW_ENTITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "rcppsw/math/range.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/types/discretize_ratio.hpp"

#include "cosm/cosm.hpp"
#include "cosm/ds/arena_grid.hpp"
#include "cosm/repr/entity2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class grid_view_entity
 * \ingroup repr
 *
 * \brief Representation of an \ref arena_grid::grid_view or \ref
 * arena_grid::const_grid_view object as an entity in the arena. This
 * representation makes it much easier to pass abstract, mult-cell objects that
 * are not entities per-se as entities for various operations such as block
 * distribution in foraging.
 *
 * It has the following characteristics:
 *
 * - Spans multiple cells in the arena.
 * - Does not "exist" in the sense that it is not detectable by robots. It lives
 *   on the same level of abstraction as the arena grid (hence the class name).
 * - Has no concept of movability/immovability (again, it is abstract).
 */
template <class T>
class grid_view_entity : public crepr::entity2D {
 public:
  grid_view_entity(const T& view, const rtypes::discretize_ratio& resolution)
      : mc_resolution(resolution), m_view(view) {}

  ~grid_view_entity(void) override = default;

  rmath::vector2u dloc2D(void) const override final { return m_view.origin()->loc(); }
  rmath::vector2d rloc2D(void) const override final {
    return rmath::uvec2dvec(m_view.origin()->loc(), mc_resolution.v());
  }

  const rtypes::discretize_ratio& resolution(void) const {
    return mc_resolution;
  }

  /**
   * \brief Get the 2D space spanned by the entity in absolute
   * coordinates in the arena in X.
   */
  rmath::ranged xspan(void) const override final {
    return rmath::ranged(rloc2D().x(),
                         rloc2D().x() + m_view.shape()[0] * mc_resolution.v());
  }

  /**
   * \brief Get the 2D space spanned by the grid_cell entity in absolute
   * coordinates in the arena in Y.
   */
  rmath::ranged yspan(void) const override final {
    return rmath::ranged(rloc2D().y(),
                         rloc2D().y() + m_view.shape()[1] * mc_resolution.v());
  }

  /**
   * \brief Determine if a real-valued point lies within the extent of the
   * entity.
   *
   * \return \c TRUE if the condition is met, and \c FALSE otherwise.
   */
  bool contains_point(const rmath::vector2d& point) const {
    return xspan().contains(point.x()) && yspan().contains(point.y());
  }

  double xdimr(void) const override { return xspan().span(); }
  double ydimr(void) const override { return yspan().span(); }

  /**
   * \brief Get the cell associated with a particular grid location within the
   * view. Asserts that both coordinates are within the bounds of the grid
   * underlying the view.
   *
   * \param i The RELATIVE X coord within the view.
   * \param j The RELATIVE Y coord within the view.
   *
   * \return A reference to the cell.
   */
  const ds::cell2D& cell(uint i, uint j) const { return m_view[i][j]; }

 protected:
  /**
   * \brief Return the size of the entity in discrete coordinates. Only suitable
   * for indexing within the entity itself.
   */
  size_t xdimd(void) const { return m_view.shape()[0]; }
  size_t ydimd(void) const { return m_view.shape()[1]; }

 private:
  /* clang-format off */
  const rtypes::discretize_ratio mc_resolution;

  T                              m_view;
  /* clang-format on */
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_GRID_VIEW_ENTITY_HPP_ */
