/**
 * \file manip_event_recorder.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <array>

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
    return { (static_cast<void>(Is), value)... };
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
