/**
 * \file cell3D_fsm.hpp
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

#ifndef INCLUDE_COSM_FSM_CELL3D_FSM_HPP_
#define INCLUDE_COSM_FSM_CELL3D_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/patterns/fsm/simple_fsm.hpp"

#include "cosm/cosm.hpp"
#include "cosm/fsm/cell3D_state.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cell3D_fsm
 * \ingroup fsm
 *
 * \brief Per-cell FSM containing the current state of the cell (empty, has a
 * block, part of the extent of a block, etc.).
 */
class cell3D_fsm final : public rpfsm::simple_fsm,
                         public rer::client<cell3D_fsm> {
 public:
  using state = cell3D_state;

  cell3D_fsm(void);
  ~cell3D_fsm(void) override = default;

  cell3D_fsm& operator=(const cell3D_fsm&) = delete;

  /**
   * \brief Initialize a COPY of a class instance via copy construction.
   *
   * This function is necessary to use this class with \ref rcppsw::ds::grid3D.
   *
   * My FSM paradigm uses *MEMBER* function pointers, so you need to initialize
   * the state map cleanly WITHOUT copy construction (even though this is the
   * copy constructor), otherwise all copies of the object will use the \p other
   * object's state map (default behavior in default copy constructor). If \p
   * other is destructed, then you will get a segfault due to dangling pointers.
   */
  cell3D_fsm(const cell3D_fsm& other);

  bool state_is_known(void) const {
    return current_state() != state::ekST_UNKNOWN;
  }
  bool state_has_block(void) const {
    return current_state() == state::ekST_HAS_BLOCK;
  }
  bool state_in_block_extent(void) const {
    return current_state() == state::ekST_BLOCK_EXTENT;
  }
  bool state_is_empty(void) const {
    return current_state() == state::ekST_EMPTY;
  }

  /* events */
  void event_unknown(void);
  void event_empty(void);
  void event_block_place(void);
  void event_block_extent(void);

 private:
  FSM_STATE_DECLARE_ND(cell3D_fsm, state_unknown);
  FSM_STATE_DECLARE_ND(cell3D_fsm, state_empty);
  FSM_STATE_DECLARE_ND(cell3D_fsm, state_block);
  FSM_STATE_DECLARE_ND(cell3D_fsm, state_block_extent);

  FSM_DEFINE_STATE_MAP_ACCESSOR(state_map, index) override {
    return &mc_state_map[index];
  }

  FSM_DECLARE_STATE_MAP(state_map, mc_state_map, state::ekST_MAX_STATES);
};

NS_END(fsm, cosm);

#endif /* INCLUDE_COSM_FSM_CELL3D_FSM_HPP_ */
