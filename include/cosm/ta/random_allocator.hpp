/**
 * \file random_allocator.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "rcppsw/common/common.hpp"
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
 * \class random_allocator
 * \ingroup ta
 *
 * \brief Allocates a task from a given set of a tasks by choosing one randomly.
 */
class random_allocator : public rer::client<random_allocator> {
 public:
  explicit random_allocator(rmath::rng* rng)
      : ER_CLIENT_INIT("cosm.ta.random_allocator"), m_rng(rng) {}

  /* Not copy constructable/assignable by default */
  random_allocator(const random_allocator&) = delete;
  const random_allocator& operator=(const random_allocator&) = delete;

  polled_task* operator()(const std::vector<polled_task*>& tasks) const {
    return tasks[m_rng->uniform(rmath::rangez(0, tasks.size() - 1))];
  }

 private:
  /* clang-format off */
  rmath::rng* m_rng;
  /* clang-format on */
};

NS_END(ta, cosm);
