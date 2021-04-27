/**
 * \file subtask_sel_probability.hpp
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

#ifndef INCLUDE_COSM_TA_SUBTASK_SEL_PROBABILITY_HPP_
#define INCLUDE_COSM_TA_SUBTASK_SEL_PROBABILITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/rng.hpp"
#include "rcppsw/math/sigmoid.hpp"

#include "cosm/ta/time_estimate.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta);

namespace config {
struct sigmoid_sel_config;
} /* namespace config */

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class subtask_sel_probability
 * \ingroup ta
 *
 * \brief Calculates the probability that a robot selects subtask 2 when it has
 * most recently executed subtask 1 (assuming partitioning is employed).
 *
 * Taken/adapted from Brutschy2014, harwell2018. Note that Brutschy2014 using
 * the interface wait time for subtask as the input time estimates, and
 * harwell2018 using the estimate of the overall execution time of
 * subtasks. Mixing the two approaches will not give good results.
 *
 * Depends on:
 *
 * - The robot's time estimates of how long it takes to complete each of the two
 *   subtasks/estimates of interface time for each subtask.
 *
 * - The reactivity parameter: how quickly should the increase in sel
 *   probability be for an increasing difference between subtask time estimates.
 *
 * - The offset parameter: how quickly should controller react to an increasing
 *   difference between subtask time estimates?
 *
 * - reactivity > 0.
 * - offset > 1.
 * - 0 < gamma < 1.
 *
 */
class subtask_sel_probability : public rer::client<subtask_sel_probability>,
                                public rmath::sigmoid {
 public:
  static constexpr const double kHARWELL2018_REACTIVITY = 8.0;
  static constexpr const double kHARWELL2018_OFFSET = 1.25;
  static constexpr const double kHARWELL2018_GAMMA = 1.0;

  static constexpr const double kBRUTSCHY2014_REACTIVITY = 1.0;
  static constexpr const double kBRUTSCHY2014_OFFSET = 8.0;
  static constexpr const double kBRUTSCHY2014_GAMMA = 0.01;

  inline static const std::string kMethodBrutschy2014 = "brutschy2014";
  inline static const std::string kMethodHarwell2018 = "harwell2018";
  inline static const std::string kMethodRandom = "random";

  /**
   * \brief Initialize subtask sel probability with default values, based
   * on whatever the selected method is.
   */
  explicit subtask_sel_probability(std::string method);
  ~subtask_sel_probability(void) override = default;

  /**
   * \brief Initialize subtask sel probability with method + parameter
   * values.
   */
  explicit subtask_sel_probability(const config::sigmoid_sel_config* config);

  const std::string& method(void) const { return mc_method; }

  /**
   * \brief After construction initialization of parameters (may be needed in
   * some situations).
   */
  void init_sigmoid(double reactivity, double offset, double gamma);

  /**
   * \brief Calculate the sel probability based on the configured method,
   * using the most recent time estimates of each subtask.
   */
  double operator()(const time_estimate* subtask1,
                    const time_estimate* subtask2,
                    rmath::rng* rng);

 private:
  /**
   * \brief Random subtask selection, regardless of time estimates.
   */
  double calc_random(rmath::rng* rng);

  /**
   * \brief Calculate the probability of switching from subtask 1 to subtask 2
   * using the piecewise method described in Brutschy2014.
   *
   * \param int_est1 Estimate of \a interface time for subtask1.
   * \param int_est2 Estimate of \a interface time for subtask2.
   *
   * \return Probability of switching.
   */
  double calc_brutschy2014(const time_estimate& int_est1,
                           const time_estimate& int_est2);

  /**
   * \brief Calculate the probability of switching from subtask 1 to subtask 2
   * using the piecewise method described in Harwell2018.
   *
   * \param exec_est1 Estimate of \a exec time for subtask1.
   * \param exec_est2 Estimate of \a exec time for subtask2.
   *
   * \return Probability of switching.
   */
  double calc_harwell2018(const time_estimate& exec_est1,
                          const time_estimate& exec_est2);

  /**
   * \brief Calculate the sigmoid activation for a pair of time estimates using
   * time estimates.
   *
   * \param est1 Exec/interface estimate 1.
   * \param est2 Exec/interface estimate 2.
   */
  double calc_sigmoid(const time_estimate& est1,
                      const time_estimate& est2) RCPPSW_PURE;

  /* clang-format off */
  const std::string mc_method;
  /* clang-format on */
};

NS_END(ta, cosm);

#endif /* INCLUDE_COSM_TA_SUBTASK_SEL_PROBABILITY_HPP_ */
