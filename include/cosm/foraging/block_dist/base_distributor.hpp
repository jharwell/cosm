/**
 * \file base_distributor.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <algorithm>
#include <vector>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/rng.hpp"
#include "rcppsw/math/vector3.hpp"

#include "cosm/ds/block3D_vector.hpp"
#include "cosm/foraging/ds/block_cluster_vector.hpp"
#include "cosm/foraging/block_dist/dist_status.hpp"
#include "cosm/foraging/block_dist/metrics/distributor_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena::ds {
class arena_grid;
} /* namespace cosm::ds */

namespace cosm::foraging::block_dist {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_distributor
 * \ingroup foraging block_dist
 *
 * \brief Base class for block distributors to enable use of strategy pattern.
 */
class base_distributor : public cfbd::metrics::distributor_metrics {
 public:
  using dist_success_cb_type = std::function<void(const crepr::sim_block3D*)>;

  /**
   * \brief How many times to attempt to distribute all blocks before giving up,
   * (possibly) causing an assertion failure on distribution.
   */
  static constexpr const size_t kMAX_DIST_TRIES = 100;

  base_distributor(cads::arena_grid* arena_grid,
                   rmath::rng* const rng)
      : m_rng(rng),
        m_arena_grid(arena_grid) {}

  virtual ~base_distributor(void) = default;

  /* Needed for use in \ref multi_cluster_distributor */
  base_distributor(const base_distributor&) = default;
  base_distributor& operator=(const base_distributor&) = delete;

  /**
   * \brief Distribute a block in the specified area by trying each random
   * distributor in turn.
   *
   * \param block The block to distribute.
   *
   * \return \c TRUE if the block distribution was successful, \c FALSE
   * otherwise.
   */
  virtual dist_status distribute_block(crepr::sim_block3D* block) = 0;

  /**
   * \brief Return a read-only list of \ref cfrepr::block_cluster objects for
   * capacity checking by external classes.
   */
  cfds::block3D_cluster_vectorro block_clustersro(void) const;
  virtual cfds::block3D_cluster_vectorno block_clustersno(void) = 0;

  /**
   * \brief Calls \ref distribute_block() on each block.
   *
   * \return \c TRUE iff all block distributions were successful, \c FALSE
   * otherwise.
   */
  virtual dist_status distribute_blocks(cds::block3D_vectorno& blocks,
                                        bool strict_success) {

    /*
     * Always try to distribute all blocks, otherwise a single failure early on
     * when distributing a given block (e.g., powerlaw distribution) can result
     * in way fewer blocks available in the arena, which throws off averaged
     * results. See COSM#145.
     */
    bool fail = false;
    for (auto *b : blocks) {
      fail |= dist_status::ekFAILURE == distribute_block(b);
    } /* for(*b..) */

    if (!strict_success || !fail) {
      return dist_status::ekSUCCESS;
    }
    return dist_status::ekFAILURE;
  }

  void clusters_update(void);

  void cluster_update_after_pickup(const crepr::sim_block3D* block,
                                   const rmath::vector2z& old_loc);
  void cluster_update_after_drop(const crepr::sim_block3D* block);

 protected:
  rmath::rng* rng(void) { return m_rng; }
  cads::arena_grid* arena_grid(void) const { return m_arena_grid; }

 private:
  /* clang-format off */
  rmath::rng*       m_rng;
  cads::arena_grid* m_arena_grid;
  /* clang-format on */
};

} /* namespace cosm::foraging::block_dist */

