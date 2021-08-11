/**
 * \file epsilon_greedy_allocator.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_TA_EPSILON_GREEDY_ALLOCATOR_HPP_
#define INCLUDE_COSM_TA_EPSILON_GREEDY_ALLOCATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <string>

#include "rcppsw/common/common.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/rng.hpp"
#include "rcppsw/rcppsw.hpp"

#include "cosm/ta/config/epsilon_greedy_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta);
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
 * From Auer2002, Pini2012
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
                          uint alloc_count) const;

 private:
  /* clang-format off */
  const config::epsilon_greedy_config* mc_config;
  rmath::rng*                           m_rng;
  /* clang-format on */
};

NS_END(ta, cosm);

#endif /* INCLUDE_COSM_TA_EPSILON_GREEDY_ALLOCATOR_HPP_ */
