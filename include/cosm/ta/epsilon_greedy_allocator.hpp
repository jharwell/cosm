/**
 * \file epsilon_greedy_allocator.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <vector>

#include "rcppsw/common/common.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/rng.hpp"
#include "rcppsw/rcppsw.hpp"

#include "cosm/ta/config/epsilon_greedy_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ta {
class polled_task;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class epsilon_greedy_allocator
 * \ingroup ta
 *
 * \brief Allocates a task from a given set of a tasks, using a randomized
 * \f$\epsilon\f$ based method with either logarithmically or linearly bounded
 * regret.
 *
 * From \cite Auer2002, \cite Pini2012
 */
class epsilon_greedy_allocator : public rer::client<epsilon_greedy_allocator> {
 public:
  static inline const std::string kRegretBoundLog = "log";
  static inline const std::string kRegretBoundLinear = "linear";

  /*
   * From Auer2002, 5.0 is considered large enough to give a logarithmic regret
   * bound.
   */
  static constexpr const double kC = 5.0;

  epsilon_greedy_allocator(const config::epsilon_greedy_config* config,
                           rmath::rng* rng)
      : ER_CLIENT_INIT("cosm.ta.epsilon_greedy_allocator"),
        mc_config(config),
        m_rng(rng) {}

  /* Not copy constructable/assignable by default */
  epsilon_greedy_allocator(const epsilon_greedy_allocator&) = delete;
  const epsilon_greedy_allocator&
  operator=(const epsilon_greedy_allocator&) = delete;

  /**
   * \brief Perform task allocation.
   *
   * \param tasks The current set of tasks.
   * \param alloc_count The total number of allocations so far.
   */
  polled_task* operator()(const std::vector<polled_task*>& tasks,
                          size_t alloc_count) const;

 private:
  /* clang-format off */
  const config::epsilon_greedy_config* mc_config;
  rmath::rng*                           m_rng;
  /* clang-format on */
};

} /* namespace cosm::ta */
