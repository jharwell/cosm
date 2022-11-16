/**
 * \file switchable_tv_generator.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/control/periodic_waveform.hpp"
#include "rcppsw/types/timestep.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, tv);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class tv::switchable_tv_generator
 * \ingroup tv
 *
 * \brief Switchable (on/off) generator to produce a temporally varying signal
 * via a configured waveform.
 */
class switchable_tv_generator {
 public:
  explicit switchable_tv_generator(
      const rcontrol::config::waveform_config* config);
  ~switchable_tv_generator(void);

  /**
   * \brief Get the applied amount of temporal variance (a percentage between 0
   * and 1) that should be applied to the robot.
   */
  double active_tv(void) const { return m_active; }

  /**
   * \brief Get the current amount of variance (a percentage between 0
   * and 1) that is configured for for the robot (regardless if it is active or
   * not).
   */
  double applied_tv(void) const { return m_applied; }

  /**
   * \brief Enable/disable application of the temporal variance.
   */
  void toggle(bool en) { m_en = en; }

  /**
   * \brief Update the value of the temporal variance in accordance with the
   * configured waveform and current timestep.
   */
  void update(const rtypes::timestep& t);

 private:
  /* clang-format off */
  bool                                     m_en{false};
  double                                   m_active{0.0};
  double                                   m_applied{0.0};
  std::unique_ptr<rcontrol::base_waveform> m_waveform;
  /* clang-format off */
};

NS_END(tv, cosm);

