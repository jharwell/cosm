/**
 * \file dispatcher.hpp
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

#ifndef INCLUDE_COSM_FORAGING_BLOCK_DIST_DISPATCHER_HPP_
#define INCLUDE_COSM_FORAGING_BLOCK_DIST_DISPATCHER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <string>
#include <memory>
#include <boost/variant.hpp>

#include "cosm/cosm.hpp"
#include "cosm/foraging/config/block_dist_config.hpp"
#include "cosm/ds/entity_vector.hpp"
#include "cosm/ds/block3D_vector.hpp"

#include "rcppsw/types/discretize_ratio.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/rng.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {
class multicell_entity;
} // namespace repr

namespace cosm::ds {
class arena_grid;
} // namespace ds

NS_START(cosm, foraging, block_dist);
class base_distributor;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class dispatcher
 * \ingroup block_dist foraging
 *
 * \brief Dispatches call to distribute blocks (or a single block), as
 * configured in simulation input file.
 *
 * - Single and dual source distribution assumes left-right rectangular arena.
 * - Power law, quad source, random distribution assume square arena.
 */
class dispatcher {
 public:
  static constexpr const char kDistSingleSrc[] = "single_source";
  static constexpr const char kDistRandom[] = "random";
  static constexpr const char kDistDualSrc[] = "dual_source";
  static constexpr const char kDistQuadSrc[] = "quad_source";
  static constexpr const char kDistPowerlaw[] = "powerlaw";

  dispatcher(cds::arena_grid* grid,
             const rtypes::discretize_ratio& resolution,
             const config::block_dist_config* config);
  ~dispatcher(void);

  dispatcher(const dispatcher&) = delete;
  dispatcher& operator=(const dispatcher&) = delete;

  /**
   * \brief Initialize the selected block distributor. This is a separate
   * function, rather than happening in the constructor, so that error handling
   * can be done without exceptions.
   *
   * \return \c TRUE if initialization successful, \c FALSE otherwise.
   */
  bool initialize(rmath::rng* rng);

  /**
   * \brief Distribute a block in the arena.
   *
   * \param block The block to distribute.
   * \param entities List of all arena entities in the arena that distribution
   * should treat as obstacles/things that blocks should not be placed in.
   *
   * \return \c TRUE iff distribution was successful, \c FALSE otherwise.
   */
  bool distribute_block(crepr::base_block3D* block, cds::const_entity_vector& entities);

  /**
   * \brief Distribute all blocks in the arena.
   *
   * \return \c TRUE iff distribution was successful, \c FALSE otherwise.
   */
  bool distribute_blocks(cds::block3D_vectorno& blocks,
                         cds::const_entity_vector& entities);

  const base_distributor* distributor(void) const {
    return m_dist.get();
  }

  const rmath::ranged& distributable_areax(void) const { return mc_arena_xrange; }
  const rmath::ranged& distributable_areay(void) const { return mc_arena_yrange; }

 private:
  /* clang-format off */
  const rtypes::discretize_ratio                mc_resolution;
  const config::block_dist_config               mc_config;
  const std::string                             mc_dist_type;

  rmath::ranged                                 mc_arena_xrange;
  rmath::ranged                                 mc_arena_yrange;

  cds::arena_grid*                              m_grid{nullptr};
  std::unique_ptr<base_distributor> m_dist;
  /* clang-format on */
};

NS_END(block_dist, foraging, cosm);

#endif /* INCLUDE_COSM_FORAGING_BLOCK_DIST_DISPATCHER_HPP_ */
