/**
 * \file cell2D.hpp
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

#ifndef INCLUDE_COSM_DS_CELL2D_HPP_
#define INCLUDE_COSM_DS_CELL2D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <string>

#include "rcppsw/math/vector2.hpp"
#include "rcppsw/patterns/decorator/decorator.hpp"
#include "rcppsw/utils/color.hpp"

#include "cosm/fsm/cell2D_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena::repr {
class base_cache;
} /* namespace cosm::arena::repr */

namespace cosm::repr {
class base_block3D;
class spatial_entity;
} /* namespace cosm::repr */

NS_START(cosm, ds);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cell2D
 * \ingroup ds
 *
 * \brief Representation of a cell on a 2D grid. A combination of FSM + handle
 * to whatever \ref repr::entity2D the cell contains, if any.
 *
 * \note Cells can maintain a color. This is a \a huge performance boost when
 * computing floor colors in ARGoS, as you no longer have to potentially query
 * \a all blocks/nests/etc only to find out a given location should be empty.
 */
class cell2D final : public rpdecorator::decorator<fsm::cell2D_fsm> {
 public:
  cell2D(void);

  /* Must be copy constructible to be able to use in \ref arena_grid */
  cell2D(const cell2D&) = default;
  cell2D& operator=(const cell2D&) = delete;

  bool operator==(const cell2D& other) const { return other.loc() == m_loc; }

  fsm::cell2D_fsm& fsm(void) { return decoratee(); }
  const fsm::cell2D_fsm& fsm(void) const { return decoratee(); }

  /* state inquiry */
  RCPPSW_DECORATE_FUNC(state_is_known, const)
  RCPPSW_DECORATE_FUNC(state_has_block, const)
  RCPPSW_DECORATE_FUNC(state_has_cache, const)
  RCPPSW_DECORATE_FUNC(state_in_cache_extent, const)
  RCPPSW_DECORATE_FUNC(state_is_empty, const)
  RCPPSW_DECORATE_FUNC(state_in_nest_extent, const)

  /**
   * \brief Reset the cell to its UNKNOWN state.
   */
  void reset(void) {
    decoratee().init();
    m_entity = nullptr;
  }

  RCPPSW_DECORATE_FUNC(block_count, const);

  /**
   * \brief Set the entity associated with this cell.
   */
  void entity(repr::spatial_entity* entity) { m_entity = entity; }
  repr::spatial_entity* entity(void) const { return m_entity; }
  repr::spatial_entity* entity(void) { return m_entity; }

  void loc(const rmath::vector2z& loc) { m_loc = loc; }
  const rmath::vector2z& loc(void) const { return m_loc; }

  const rutils::color& color(void) const { return m_color; }
  void color(const rutils::color& color) { m_color = color; }

  /**
   * \brief Get the block entity associated with this cell.
   *
   * Will be NULL unless it contains a block, so check the cell's state before
   * calling this function.
   */
  crepr::base_block3D* block3D(void) const RCSW_PURE;
  crepr::base_block3D* block3D(void) RCSW_PURE;

  /**
   * \brief Get the cache entity associated with this cell.
   *
   * Will be NULL unless it contains a cache, so check the cell's state before
   * calling this function.
   */
  carepr::base_cache* cache(void) const RCSW_PURE;
  carepr::base_cache* cache(void) RCSW_PURE;

 private:
  /* clang-format off */
  repr::spatial_entity* m_entity{nullptr};
  rmath::vector2z       m_loc{};
  rutils::color         m_color{rutils::color::kWHITE};
  /* clang-format on */
};

NS_END(ds, cosm);

#endif /* INCLUDE_COSM_DS_CELL2D_HPP_ */
