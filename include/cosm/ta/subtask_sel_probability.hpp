/**
 * \file subtask_sel_probability.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

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
namespace cosm::ta {

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
 * Adapted from \cite TA:Brutschy2014-taskseq,\cite Harwell:2018-partitioning.
 * Note that \cite TA:Brutschy2014-taskseq using the interface wait time for
 * subtask as the input time estimates, and \cite Harwell:2018-partitioning
 * using the estimate of the overall execution time of subtasks. Mixing the two
 * approaches will not give good results.
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
class subtask_sel_probability final : public rer::client<subtask_sel_probability>,
                                      public rmath::sigmoid {
 public:
  static constexpr const double kHARWELL2018_REACTIVITY = 8.0;
  static constexpr const double kHARWELL2018_OFFSET = 1.25;
  static constexpr const double kHARWELL2018_GAMMA = 1.0;

  static constexpr const double kBRUTSCHY2014_REACTIVITY = 1.0;
  static constexpr const double kBRUTSCHY2014_OFFSET = 8.0;
  static constexpr const double kBRUTSCHY2014_GAMMA = 0.01;

  static inline const std::string kMethodBrutschy2014 = "brutschy2014";
  static inline const std::string kMethodHarwell2018 = "harwell2018";
  static inline const std::string kMethodRandom = "random";

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

} /* namespace cosm::ta */
