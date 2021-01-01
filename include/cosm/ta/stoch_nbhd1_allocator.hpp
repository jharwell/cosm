/**
 * \file stoch_nbhd1_allocator.hpp
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

#ifndef INCLUDE_COSM_TA_STOCH_NBHD1_ALLOCATOR_HPP_
#define INCLUDE_COSM_TA_STOCH_NBHD1_ALLOCATOR_HPP_

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
 * from Harwell2020a.
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

#endif /* INCLUDE_COSM_TA_STOCH_NBHD1_ALLOCATOR_HPP_ */
