/**
 * \file abort_probability.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/sigmoid.hpp"
#include "rcppsw/rcppsw.hpp"
#include "rcppsw/types/timestep.hpp"

#include "cosm/ta/time_estimate.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class abort_probability
 * \ingroup ta
 *
 * \brief Calculates the probability that a robot will abort the task it is
 * currently working on using the negative exponential distribution.
 *
 * Reactivity and offset are assumed to both be > 0.
 *
 * Depends on:
 *
 * - The reactivity parameter: How sensitive should controller be to abrupt changes
 *   in task estimates/execution times?
 *
 * - The offset parameter: How much the current_exec/prev_estimate ratio will be
 *   allowed to grow before causing the probability to grow exponentially.
 *
 * - A time estimate for the task (can execution time thus far, interface time
 *   thus far, etc.).
 */
class abort_probability : public rmath::sigmoid {
 public:
  /*
   * A default reactivity value found experimentally to work well.
   */
  static constexpr const double kDEFAULT_REACTIVITY = 8.0;

  /*
   * A default offset value found experimentally to work well.
   */
  static constexpr const double kDEFAULT_OFFSET = 3.0;

  /*
   * A default gamma value because there needs to be one.
   */
  static constexpr const double kDEFAULT_GAMMA = 1.0;

  /*
   * \brief All tasks need to have a small abort probability, so that they don't
   * get stuck indefinitely.
   */
  static constexpr const double kMIN_ABORT_PROB = 0.0001;

  /**
   * \brief Initialize an abort probability calculation with default values.
   */
  abort_probability(void)
      : sigmoid(kDEFAULT_REACTIVITY, kDEFAULT_OFFSET, kDEFAULT_GAMMA) {}

  /**
   * \brief Initialize abort probability calculation with user-specified values.
   */
  explicit abort_probability(const rmath::config::sigmoid_config* config);

  /**
   * \brief Calculate the current abort probability, based on the most recent
   * estimate of task execution time and the currently elapsed time spent on the
   * the task.
   *
   * \param exec_time Current execution time.
   * \param whole_task Most recent task estimate.
   *
   * \return The abort probability.
   */
  double operator()(const rtypes::timestep& exec_time,
                    const time_estimate& whole_task);
  double calc(const rtypes::timestep& exec_time,
              const time_estimate& whole_task) {
    return operator()(exec_time, whole_task);
  }
};

NS_END(ta, cosm);

