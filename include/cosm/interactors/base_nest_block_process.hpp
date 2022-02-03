/**
 * \file base_nest_block_process.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_INTERACTORS_BASE_NEST_BLOCK_PROCESS_HPP_
#define INCLUDE_COSM_INTERACTORS_BASE_NEST_BLOCK_PROCESS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/mpl/at.hpp>

#include "rcppsw/er/client.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, interactors);

/*******************************************************************************
 * Classes
 ******************************************************************************/

/**
 * \class base_nest_block_process
 * \ingroup interactors
 *
 * \brief Base class providing common functionality for handling a robot's nest
 * block drop.
 */
template <typename TController,>
class base_nest_block_process {
 public:
  using metrics_manager_type = typename controller_spec::metrics_manager_type;

  explicit base_nest_block_process(metrics_manager_type* const metrics_manager)
      : m_metrics_manager(metrics_manager) {}

  virtual ~base_nest_block_process(void) = default;

  base_nest_block_process(base_nest_block_process&&) = default;

  /* Not copy-constructible/assignable by default. */
  base_nest_block_process(const base_nest_block_process&) = delete;
  base_nest_block_process& operator=(const base_nest_block_process&) = delete;

 protected:
  metrics_manager_type* metrics_manager(void) const { return m_metrics_manager; }

 private:
  /* clang-format off */
  metrics_manager_type* const  m_metrics_manager;
  /* clang-format on */
};

NS_END(interactors, cosm);

#endif /* INCLUDE_COSM_INTERACTORS_BASE_NEST_BLOCK_PROCESS_HPP_ */
