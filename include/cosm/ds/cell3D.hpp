/**
 * \file cell3D.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

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
class sim_block3D;
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
  cell3D(void);

  cell3D(const cell3D&) = default;
  cell3D& operator=(const cell3D&) = delete;

  bool operator==(const cell3D& other) const { return other.loc() == m_loc; }

  fsm::cell3D_fsm& fsm(void) { return decoratee(); }
  const fsm::cell3D_fsm& fsm(void) const { return decoratee(); }

  /* state inquiry */
  RCPPSW_DECORATE_DECLDEF(state_is_empty, const)
  RCPPSW_DECORATE_DECLDEF(state_has_block, const)
  RCPPSW_DECORATE_DECLDEF(state_in_block_extent, const)

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

  void loc(const rmath::vector3z& loc) { m_loc = loc; }
  const rmath::vector3z& loc(void) const { return m_loc; }

  crepr::sim_block3D* block(void) const RCPPSW_PURE;
  crepr::sim_block3D* block(void) RCPPSW_PURE;

 private:
  /* clang-format off */
  crepr::entity3D* m_entity{nullptr};
  rmath::vector3z  m_loc{};
  /* clang-format on */
};

NS_END(ds, cosm);
