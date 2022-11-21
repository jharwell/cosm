/**
 * \file partition_probability.hpp
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
#include "rcppsw/math/expression.hpp"
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
 * \class partition_probability
 * \ingroup ta
 *
 * \brief Calculates the probability that a robot partitions its current task
 * using the negative exponential distribution.
 *
 * Reactivity is assumed to be > 0.
 * Offset is assumed to be > 0.
 *
 * Depends on:
 *
 * - The robot's time estimates of how long it takes to complete each of the two
 *   subtasks, as well as an estimate of how long it takes to complete the
 *   unpartitioned task.
 *
 * - The reactivity parameter: how sensitive should controller be to abrupt
 *   changes in the estimates?
 *
 * Used in \cite Harwell:2018-partitioning, \cite Harwell:2020a-demystify.
 * Originally inspired by \cite Pini2011b.
 */
class partition_probability final : public rmath::sigmoid,
                                    public rer::client<partition_probability> {
 public:
  static inline const std::string kMethodPini2011 = "pini2011";
  static inline const std::string kMethodRandom = "random";

  /*
   * A default reactivity value determined experimentally to work well.
   */
  static constexpr const double kDEFAULT_REACTIVITY = 1.5;

  /*
   * A default reactivity value that does *not* induce singularities in the
   * overall equation. Choose not to employ it at your own risk...
   */
  static constexpr const double kDEFAULT_OFFSET = 1.0;

  /* A default gamma value because there needs to be one */
  static constexpr const double kDEFAULT_GAMMA = 1.0;

  /**
   * \brief Initialize partitioning probability with default values based on
   * whatever the selected method is.
   */
  explicit partition_probability(const std::string& method)
      : sigmoid(kDEFAULT_REACTIVITY, kDEFAULT_OFFSET, kDEFAULT_GAMMA),
        ER_CLIENT_INIT("cosm.ta.partition_probability"),
        mc_method(method) {}

  /**
   * \brief Initialize partitioning probability explicity with method +
   * parameter values.
   */
  explicit partition_probability(const config::sigmoid_sel_config* config);

  double operator()(const time_estimate& task,
                    const time_estimate& subtask1,
                    const time_estimate& subtask2,
                    rmath::rng* rng);

  const std::string& method(void) const { return mc_method; }

 private:
  double calc_pini2011(const time_estimate& task,
                       const time_estimate& subtask1,
                       const time_estimate& subtask2);

  double calc_random(rmath::rng* rng);

  /* clang-format off */
  const std::string mc_method;
  /* clang-format on */
};

} /* namespace cosm::ta */
