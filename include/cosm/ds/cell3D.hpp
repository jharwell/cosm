/**
 * \file cell3D.hpp
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

#ifndef INCLUDE_COSM_STRUCTURE_CELL3D_HPP_
#define INCLUDE_COSM_STRUCTURE_CELL3D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <string>

#include "rcppsw/math/vector3.hpp"
#include "rcppsw/patterns/decorator/decorator.hpp"

#include "cosm/cosm.hpp"
#include "cosm/fsm/cell3D_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {
class cube_block3D;
class ramp_block3D;
class entity3D;
} /* namespace cosm::repr */

NS_START(cosm, ds);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cell3D
 * \ingroup ds
 *
 * \brief Representation of a cell on a 3D grid. A combination of FSM + handle
 * to whatever \ref repr::entity3D the cell contains, if any.
 */
class cell3D final : public rpdecorator::decorator<fsm::cell3D_fsm> {
 public:
  /* must have no parameters to be used with \ref structure3D */
  cell3D(void);

  cell3D(const cell3D&) = default;
  cell3D& operator=(const cell3D&) = delete;

  bool operator==(const cell3D& other) const { return other.loc() == m_loc; }

  fsm::cell3D_fsm& fsm(void) { return decoratee(); }
  const fsm::cell3D_fsm& fsm(void) const { return decoratee(); }

  /* state inquiry */
  RCPPSW_DECORATE_FUNC(state_is_empty, const)
  RCPPSW_DECORATE_FUNC(state_has_block, const)
  RCPPSW_DECORATE_FUNC(state_in_block_extent, const)

  /**
   * \brief Reset the cell to its UNKNOWN state.
   */
  void reset(void) {
    decoratee().init();
    m_entity = nullptr;
  }

  /**
   * \brief Set the entity associated with this cell.
   */
  void entity(crepr::entity3D* entity) { m_entity = entity; }
  const crepr::entity3D* entity(void) const { return m_entity; }

  void loc(const rmath::vector3u& loc) { m_loc = loc; }
  const rmath::vector3u& loc(void) const { return m_loc; }

  /**
   * \brief Get the cube block entity associated with this cell.
   *
   * Will be NULL unless it contains a cube block, so check the cell's state
   * before calling this function.
   */
  crepr::cube_block3D* cube_block(void) const RCSW_PURE;
  crepr::cube_block3D* cube_block(void) RCSW_PURE;

  /**
   * \brief Get the ramp block entity associated with this cell.
   *
   * Will be NULL unless it contains a ramp block, so check the cell's state
   * before calling this function.
   */
  crepr::ramp_block3D* ramp_block(void) const RCSW_PURE;
  crepr::ramp_block3D* ramp_block(void) RCSW_PURE;

 private:
  /* clang-format off */
  crepr::entity3D* m_entity{nullptr};
  rmath::vector3u  m_loc{};
  /* clang-format on */
};

NS_END(ds, cosm);

#endif /* INCLUDE_COSM_STRUCTURE_CELL3D_HPP_ */
