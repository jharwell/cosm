/**
 * \file governed_diff_drive.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/kin2D/governed_diff_drive.hpp"

#include <numeric>

#include "cosm/tv/switchable_tv_generator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, kin2D);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
double governed_diff_drive::active_throttle(void) const {
  auto sum = std::accumulate(std::begin(m_generators),
                             std::end(m_generators),
                             0.0,
                             [&](double accum, const auto* generator) {
                               return accum + generator->active_tv();
                             });
  /* active throttle cannot be negative */
  return std::max(sum, 0.0);
}

double governed_diff_drive::applied_throttle(void) const {
  auto sum = std::accumulate(std::begin(m_generators),
                             std::end(m_generators),
                             0.0,
                             [&](double accum, const auto* generator) {
                               return accum + generator->applied_tv();
                             });
  /* applied throttle cannot be negative */
  return std::max(sum, 0.0);
}
NS_END(kin2d, cosm);
