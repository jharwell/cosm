/**
 * \file supervisor_fsm.hpp
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

#ifndef INCLUDE_COSM_FSM_SUPERVISOR_FSM_HPP_
#define INCLUDE_COSM_FSM_SUPERVISOR_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <utility>
#include <boost/variant.hpp>

#include "cosm/cosm.hpp"
#include "rcppsw/patterns/fsm/hfsm.hpp"
#include "cosm/ta/taskable.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm);

namespace subsystem {
class saa_subsystem2D;
} /* namespace subsystem */

namespace ta {
class base_executive;
} /* namespace ta */

NS_START(fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class supervisor_fsm
 * \ingroup fsm
 *
 * \brief An FSM used supervise robot operation.
 *
 * In normal operation, the \ref taskable object is just executed. If a
 * non-standard event is received (implemented in derived classes), then
 * normal operation can be replaced by something else (e.g. stopping all robot
 * motion if a robot malfunction event is received).
 */
class supervisor_fsm final : public rpfsm::hfsm,
                             public rer::client<supervisor_fsm> {
  using variant_type = boost::variant<ta::taskable*, ta::base_executive*>;

 public:
  supervisor_fsm(const variant_type& variant, subsystem::saa_subsystem2D* saa);

  supervisor_fsm& operator=(const supervisor_fsm&) = delete;
  supervisor_fsm(const supervisor_fsm&) = delete;

  /**
   * \brief Signal that the  \ref taskable object should not be run every
   * timestep until the robot has been repaired.
   */
  void event_malfunction(void);

  /**
   * \brief Signal that the  \ref taskable object should not be run every
   * timestep until the robot has been repaired.
   */
  void event_repair(void);

  template<typename T>
  void supervisee_update(T* h) { m_variant = variant_type(h); }

  void run(void) {
      inject_event(rpfsm::event_signal::ekRUN, rpfsm::event_type::ekNORMAL);
  }

 private:
  enum states {
    ekST_START,
    /**
     * Normal operation.
     */
    ekST_NORMAL,

    /**
     * Non-normal operation: the robot has malfunctioned
     */
    ekST_MALFUNCTION,

    ekST_MAX_STATES
  };

  /* supervisor states */
  HFSM_STATE_DECLARE_ND(supervisor_fsm, start);
  HFSM_STATE_DECLARE_ND(supervisor_fsm, normal);
  HFSM_STATE_DECLARE_ND(supervisor_fsm, malfunction);

  HFSM_DEFINE_STATE_MAP_ACCESSOR(state_map, index) override {
    return &mc_state_map[index];
  }

  HFSM_DECLARE_STATE_MAP(state_map, mc_state_map, states::ekST_MAX_STATES);

  /* clang-format off */
  variant_type                      m_variant;
  subsystem::saa_subsystem2D* const m_saa;
  /* clang-format on */
};

NS_END(fsm, cosm);

#endif /* INCLUDE_COSM_FSM_SUPERVISOR_FSM_HPP_ */