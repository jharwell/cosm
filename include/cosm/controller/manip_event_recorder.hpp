/**
 * \file manip_event_recorder.hpp
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

#ifndef INCLUDE_COSM_CONTROLLER_MANIP_EVENT_RECORDER_HPP_
#define INCLUDE_COSM_CONTROLLER_MANIP_EVENT_RECORDER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/types/timestep.hpp"

#include "cosm/controller/metrics/manipulation_metrics.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, controller);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class manip_event_recorder
 * \ingroup controller
 *
 * \brief Records the state of various events within a single timestep in a
 * controller as they manipulate their environment.
 */
template <size_t MaxEvents>
class manip_event_recorder : public ccmetrics::manipulation_metrics {
 public:
  manip_event_recorder(void) = default;
  ~manip_event_recorder(void) = default;

  /* manipulation metrics */
  bool status(uint index) const override { return m_events[index]; }
  rtypes::timestep penalty(uint index) const override {
    return m_penalties[index];
  }

  /**
   * \brief Record that the specified event resulted in the specified penalty
   * when it occurred this timestep.
   */
  void record(uint index, const rtypes::timestep& penalty) {
    m_events[index] = true;
    m_penalties[index] = penalty;
  }

  void reset(void) {
    m_events.fill(false);
    m_penalties.fill(rtypes::timestep(0));
  }

 private:
  template <typename T, size_t... Is>
  std::array<T, sizeof...(Is)> make_array(const T& value,
                                          std::index_sequence<Is...>) {
    return {(static_cast<void>(Is), value)...};
  }

  template <size_t N, typename T>
  std::array<T, N> make_array(const T& value) {
    return make_array(value, std::make_index_sequence<N>());
  }

  /* clang-format off */
  std::array<rtypes::timestep, MaxEvents> m_penalties{make_array<MaxEvents>(rtypes::timestep(0))};
  std::array<bool, MaxEvents>             m_events{};
  /* clang-format on */
};

NS_END(controller, cosm);

#endif /* INCLUDE_COSM_CONTROLLER_MANIP_EVENT_RECORDER_HPP_ */
