/**
 * \file stoch_nbhd1_allocator.hpp
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

namespace ds {
class bi_tdgraph;
} /* namespace ds */

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class stoch_nbhd1_allocator
 * \ingroup ta
 *
 * \brief Allocates a task from a \ref bi_tdgraph using the STOCH-NBHD1 method
 * from \cite Harwell2020a-demystify.
 */
class stoch_nbhd1_allocator : public rer::client<stoch_nbhd1_allocator> {
 public:
  stoch_nbhd1_allocator(rmath::rng* rng, ds::bi_tdgraph* graph)
      : ER_CLIENT_INIT("cosm.ta.stoch_nbhd1_allocator"),
        m_rng(rng),
        m_graph(graph) {}

  /* Not copy constructable/assignable by default */
  stoch_nbhd1_allocator(const stoch_nbhd1_allocator&) = delete;
  const stoch_nbhd1_allocator&
  operator=(const stoch_nbhd1_allocator& other) = delete;

  polled_task* operator()(const polled_task* current_task) const;

 private:
  /* clang-format off */
  rmath::rng*     m_rng;
  ds::bi_tdgraph* m_graph;
  /* clang-format on */
};

NS_END(ta, cosm);
