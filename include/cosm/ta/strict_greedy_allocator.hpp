/**
 * \file strict_greedy_allocator.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "rcppsw/common/common.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/rng.hpp"
#include "rcppsw/rcppsw.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta);
class polled_task;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class strict_greedy_allocator
 * \ingroup ta
 *
 * \brief Allocates a task from a given set of a tasks, using a strict greedy
 * approach: always taking the task of minimum cost. Provably optimal task
 * allocation policy if the task allocation space is a matroid, (probably)
 * suboptimal otherwise.
 */
class strict_greedy_allocator : public rer::client<strict_greedy_allocator> {
 public:
  explicit strict_greedy_allocator(rmath::rng* rng)
      : ER_CLIENT_INIT("cosm.ta.strict_greedy_allocator"), m_rng(rng) {}

  /* Not copy constructable/assignable by default */
  strict_greedy_allocator(const strict_greedy_allocator&) = delete;
  const strict_greedy_allocator&
  operator=(const strict_greedy_allocator&) = delete;

  /**
   * \brief Perform task allocation.
   *
   * \param tasks The current set of tasks.
   */
  polled_task* operator()(const std::vector<polled_task*>& tasks) const;

 private:
  /* clang-format off */
  rmath::rng* m_rng;
  /* clang-format on */
};

NS_END(ta, cosm);

