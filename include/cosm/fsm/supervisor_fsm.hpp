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
#include <boost/variant.hpp>
#include <utility>

#include "rcppsw/patterns/fsm/simple_fsm.hpp"

#include "cosm/cosm.hpp"
#include "cosm/ta/taskable.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm);

namespace subsystem {
class saa_subsystemQ3D;
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
class supervisor_fsm final : public rpfsm::simple_fsm,
                             public rer::client<supervisor_fsm> {
  using supervisee_variant_type =
      boost::variant<ta::taskable*, ta::base_executive*>;

 public:
  explicit supervisor_fsm(subsystem::saa_subsystemQ3D* saa);

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

  template <typename T>
  void supervisee_update(T* h) {
    m_supervisee = supervisee_variant_type(h);
  }

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
  FSM_STATE_DECLARE_ND(supervisor_fsm, start);
  FSM_STATE_DECLARE_ND(supervisor_fsm, normal);
  FSM_STATE_DECLARE_ND(supervisor_fsm, malfunction);

  FSM_DEFINE_STATE_MAP_ACCESSOR(state_map, index) override {
    return &mc_state_map[index];
  }

  FSM_DECLARE_STATE_MAP(state_map, mc_state_map, states::ekST_MAX_STATES);

  /* clang-format off */
  subsystem::saa_subsystemQ3D* m_saa;
  supervisee_variant_type      m_supervisee{};
  /* clang-format on */
};

NS_END(fsm, cosm);

#endif /* INCLUDE_COSM_FSM_SUPERVISOR_FSM_HPP_ */
