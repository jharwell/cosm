/**
 * \file stoch_nbhd1_allocator.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ta/stoch_nbhd1_allocator.hpp"

#include <algorithm>

#include "cosm/ta/ds/bi_tdgraph.hpp"
#include "cosm/ta/polled_task.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
polled_task*
stoch_nbhd1_allocator::operator()(const polled_task* current_task) const {
  /*
   * If there is no active TAB, then the root task is not partitionable, so
   * return it.
   */
  if (nullptr == m_graph->active_tab()) {
    return m_graph->root();
  } else {
    /*
     * Update our active TAB so that we perform partitioning from the correct
     * place. We have to pass in the current_task(), because the TAB's active
     * task has already been updated to be NULL after the task finish/abort
     * that brought us to this function.
     */
    m_graph->active_tab_update(current_task, m_rng);

    return m_graph->active_tab()->task_allocate(m_rng);
  }
} /* alloc_stoch_nbhd1() */

NS_END(ta, cosm);
