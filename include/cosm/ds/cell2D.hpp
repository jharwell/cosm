/**
 * \file cell2D.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

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
class sim_block3D;
class spatial_entity;
} /* namespace cosm::repr */

namespace cosm::ds {

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
  RCPPSW_DECORATE_DECLDEF(state_is_known, const)
  RCPPSW_DECORATE_DECLDEF(state_has_block, const)
  RCPPSW_DECORATE_DECLDEF(state_has_cache, const)
  RCPPSW_DECORATE_DECLDEF(state_in_block_extent, const)
  RCPPSW_DECORATE_DECLDEF(state_in_cache_extent, const)
  RCPPSW_DECORATE_DECLDEF(state_is_empty, const)
  RCPPSW_DECORATE_DECLDEF(state_in_nest_extent, const)

  /**
   * \brief Reset the cell to its UNKNOWN state.
   */
  void reset(void) {
    decoratee().init();
    m_entity = nullptr;
  }

  RCPPSW_DECORATE_DECLDEF(block_count, const);

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
  crepr::sim_block3D* block3D(void) const RCPPSW_PURE;
  crepr::sim_block3D* block3D(void) RCPPSW_PURE;

  /**
   * \brief Get the cache entity associated with this cell.
   *
   * Will be NULL unless it contains a cache, so check the cell's state before
   * calling this function.
   */
  carepr::base_cache* cache(void) const RCPPSW_PURE;
  carepr::base_cache* cache(void) RCPPSW_PURE;

 private:
  /* clang-format off */
  repr::spatial_entity* m_entity{nullptr};
  rmath::vector2z       m_loc{};
  rutils::color         m_color{rutils::color::kWHITE};
  /* clang-format on */
};

} /* namespace cosm::ds */
