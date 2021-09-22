/**
 * \file dist_status.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * This file is part of COSM.
 *
 * COSM is free software: you can redist it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * COSM is distd in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * COSM.  If not, see <http://www.gnu.org/licenses/
 */

#ifndef INCLUDE_COSM_FORAGING_BLOCK_DIST_DIST_STATUS_HPP_
#define INCLUDE_COSM_FORAGING_BLOCK_DIST_DIST_STATUS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, foraging, block_dist);

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
/**
 * \brief The distribution status of \a ALL blocks that are distributed at once
 * via \ref dispatcher.
 */

enum class dist_status {
  /**
   * \brief All blocks/a given block were distributed successfully.
   */
  ekSUCCESS,

  /**
   * \brief Fatal failure to distribute one or more blocks.
   *
   */
  ekFAILURE
};

NS_END(block_dist, foraging, cosm);

#endif /* INCLUDE_COSM_FORAGING_BLOCK_DIST_DIST_STATUS_HPP_ */
