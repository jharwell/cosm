/**
 * \file base_distributor.hpp
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

#ifndef INCLUDE_COSM_FORAGING_BLOCK_DIST_BASE_DISTRIBUTOR_HPP_
#define INCLUDE_COSM_FORAGING_BLOCK_DIST_BASE_DISTRIBUTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <algorithm>
#include <vector>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/rng.hpp"

#include "cosm/ds/block3D_vector.hpp"
#include "cosm/ds/entity_vector.hpp"
#include "cosm/foraging/ds/block_cluster_vector.hpp"
#include "cosm/foraging/block_dist/dist_status.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, block_dist);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_distributor
 * \ingroup foraging block_dist
 *
 * \brief Base class for block distributors to enable use of strategy pattern.
 */
class base_distributor {
 public:
  /**
   * \brief How many times to attempt to distribute all blocks before giving up,
   * causing an assertion failure on distribution.
   */
  static constexpr const uint kMAX_DIST_TRIES = 1000;

  explicit base_distributor(rmath::rng* const rng) : m_rng(rng) {}
  virtual ~base_distributor(void) = default;

  /* Needed for use in \ref multi_cluster_distributor */
  base_distributor(const base_distributor&) = default;
  base_distributor& operator=(const base_distributor&) = delete;

  /**
   * \brief Distribute a block in the specified area by trying each random
   * distributor in turn.
   *
   * \param block The block to distribute.
   * \param entities The list of entities that the block should be distributed
   *                 around. If block distribution is successful, then the
   *                 distributed block is added to the entity list.
   *
   * \return \c TRUE if the block distribution was successful, \c FALSE
   * otherwise.
   */
  virtual dist_status distribute_block(crepr::base_block3D* block,
                                       cds::const_spatial_entity_vector& entities) = 0;

  /**
   * \brief Return a read-only list of \ref block_clusters for capacity checking
   * by external classes.
   */
  virtual cfds::block3D_cluster_vector block_clusters(void) const = 0;

  /**
   * \brief Calls \ref distribute_block() on each block.
   *
   * \return \c TRUE iff all block distributions were successful, \c FALSE
   * otherwise.
   */
  virtual dist_status distribute_blocks(cds::block3D_vectorno& blocks,
                                        cds::const_spatial_entity_vector& entities,
                                        bool strict_success) {
    auto status = std::all_of(blocks.begin(),
                              blocks.end(),
                              [&](auto& b) {
                                return dist_status::ekSUCCESS == distribute_block(b, entities);
                              });
    if (!strict_success || status) {
      return dist_status::ekSUCCESS;
    }
    return dist_status::ekFAILURE;
  }

  rmath::rng* rng(void) { return m_rng; }

 private:
  /* clang-format off */
  rmath::rng* m_rng;
  /* clang-format on */
};

NS_END(block_dist, foraging, cosm);

#endif /* INCLUDE_COSM_FORAGING_BLOCK_DIST_BASE_DISTRIBUTOR_HPP_ */
