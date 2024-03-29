/**
 * \file base_cache.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <algorithm>
#include <memory>

#include "rcppsw/patterns/prototype/clonable.hpp"
#include "rcppsw/spatial/euclidean_dist.hpp"
#include "rcppsw/types/timestep.hpp"
#include "rcppsw/math/rng.hpp"

#include "cosm/repr/colored_entity.hpp"
#include "cosm/repr/unicell_immovable_entity2D.hpp"

#include "cosm/ds/block3D_ht.hpp"
#include "cosm/ds/block3D_vector.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena::repr {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_cache
 * \ingroup arena repr
 *
 * \brief Base class for representating a cache within the arena. Caches do not
 * have state, and if/when a cache becomes empty, it needs to be deleted by an
 * enclosing class. Caches have both real (where they actually live in the
 * world) and discretized locations (where they are mapped to within the arena
 * map).
 */
class base_cache : public crepr::unicell_immovable_entity2D,
                   public rer::client<base_cache>,
                   public crepr::colored_entity,
                   public rpprototype::clonable<base_cache> {
 public:
  /**
   * \param dimension The size of the cache. Does not have to be a multiple of
   *                   the arena resolution, but doing so makes it easier.
   * \param resolution The arena resolution.
   * \param center (X,Y) coordinates of the center of the cache.
   * \param blocks The initial block list for the cache.
   * \param id The ID to assign to the cache; -1 for a new cache, which
   *           will generate a new ID, or any positive # to use the same ID as
   *           an existing cache (used when cloning a cache into a robot's
   *           perception).
   */
  struct params {
    /* clang-format off */
    rspatial::euclidean_dist          dimension; /* caches are square */
    rtypes::discretize_ratio      resolution;
    rmath::vector2d               center;
    const cds::block3D_vectorno&& blocks;
    rtypes::type_uuid             id;
    /* clang-format on */
  };
  /**
   * \brief The minimum # of blocks required for a cache to exist (less than
   * this and you just have a bunch of blocks).
   */
  static constexpr const size_t kMinBlocks = 2;

  explicit base_cache(const params& p);
  ~base_cache(void) override = default;

  /**
   * \brief Disallow direct object comparisons, because we may want to compare
   * for equality in terms of IDs or object locations, and it is better to
   * require explicit comparisons for BOTH, rather than just one. It also makes
   * it unecessary to have to remember which type the comparison operator==()
   * does for this class.
   */
  bool operator==(const base_cache& other) const = delete;

  /**
   * \brief Compare two \ref base_cache objects for equality based on their ID.
   */
  bool idcmp(const base_cache& other) const { return this->id() == other.id(); }

  /**
   * \brief Compare two \ref base_cache objects for equality based on their
   * discrete center.
   */
  bool dloccmp(base_cache& other) {
    return this->dcenter2D() == other.dcenter2D();
  }

  void blocks_map_enable(void);

  bool contains_block(const crepr::sim_block3D* c_block) const RCPPSW_PURE;
  size_t n_blocks(void) const { return m_blocks_vec.size(); }

  /**
   * \brief Get a list of the blocks currently in the cache.
   */
  cds::block3D_vectorno& blocks(void) { return m_blocks_vec; }
  const cds::block3D_vectorno& blocks(void) const { return m_blocks_vec; }


  /**
   * \brief Add a new block to the cache's list of blocks.
   *
   * Does not update the block's location.
   */
  void block_add(crepr::sim_block3D* block);

  /**
   * \brief Remove a block from the cache's list of blocks, without modifying
   * it.
   */
  void block_remove(const crepr::sim_block3D* victim);

  /**
   * \brief Get a random block from the cache.
   */
  crepr::sim_block3D* block_select(rmath::rng* rng);

  std::unique_ptr<base_cache> clone(void) const override final;

  rtypes::timestep creation_ts(void) const { return m_creation; }
  void creation_ts(const rtypes::timestep& ts) { m_creation = ts; }

 private:
  /* clang-format off */
  static int                     m_next_id;

  const rtypes::discretize_ratio mc_resolution;

  bool                           m_map_en{false};
  rtypes::timestep               m_creation{0};

  /**
   * \brief Map of blocks the cache contains. Allows for fast checking of "Does
   * this cache contain this block?", but (much) slower clone()ing, so it is
   * disabled by default.
   */
  cds::block3D_htno              m_blocks_map{};

  /**
   * \brief Vector of blocks the cache contains. Allows for fast clone()ing, but
   * slower checking of "Does this cache contain this block?".
   */
  cds::block3D_vectorno          m_blocks_vec{};
  /* clang-format on */
};

} /* namespace cosm::arena::repr */

