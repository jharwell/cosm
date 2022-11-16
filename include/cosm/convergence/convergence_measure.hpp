/**
 * \file convergence_measure.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <algorithm>
#include <cmath>
#include <limits>

#include "rcppsw/math/expression.hpp"
#include "rcppsw/math/normalize.hpp"
#include "rcppsw/rcppsw.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, convergence);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class convergence_measure
 * \ingroup convergence
 *
 * \brief Base class for all convergence measurements that can be computed on
 * swarms. Provides common functionality needed by all derived classes (cannot
 * be used to do anything useful on its own).
 */
class convergence_measure : public rmath::expression<double> {
 public:
  explicit convergence_measure(double epsilon) : mc_epsilon(epsilon) {}

  /**
   * \brief Set the raw value of the measure, computed at a given time.
   *
   * Also updates the min/max raw values that have been seen so far/since the
   * last reset.
   */
  void update_raw(double value) {
    m_raw = value;
    m_raw_min = std::min(m_raw, m_raw_min);
    m_raw_max = std::max(m_raw, m_raw_max);
  }
  /**
   * \brief Return the raw value of the convergence measure.
   */
  double raw(void) const { return m_raw; }
  double raw_min(void) const { return m_raw_min; }
  double raw_max(void) const { return m_raw_max; }

  /**
   * \brief Set the normalized value of the measure, computed at a given
   * time. Any normalization calculation should be performed prior to calling
   * this function (i.e. this function does not perform normalization itself, so
   * you should not pass \ref raw()).
   */
  void set_norm(double value) { eval(value); }

  void reset(void) {
    rmath::expression<double>::reset();
    m_raw = 0.0;
    m_raw_min = std::numeric_limits<double>::max();
    m_raw_max = std::numeric_limits<double>::min();
    m_converged = false;
    m_norm_prev = 0.0;
  }

  /**
   * \brief Update the state of the convergence measure. Prior to calling this
   * function in a derived class, the raw/normalized values need to have been
   * calculated and the min/max updated.
   *
   * \return \c TRUE iff convergence has been achieved according to configured
   * parameters and the current state of the swarm.
   */
  bool update_convergence_state(void) {
    m_converged = (std::fabs(v() - m_norm_prev) <= mc_epsilon);
    m_norm_prev = v();
    return m_converged;
  }

  /**
   * \brief Return \c TRUE iff the measure is currently converged (i.e. has been
   * above the threshold for at least the specified length).
   */
  bool converged(void) const { return m_converged; }

 private:
  /* clang-format off */
  const double mc_epsilon{0.0};

  double m_norm_prev{0.0};
  bool   m_converged{false};

  double m_raw{0.0};
  double m_raw_min{std::numeric_limits<double>::max()};
  double m_raw_max{std::numeric_limits<double>::min()};
  /* clang-format on */
};

NS_END(convergence, cosm);
