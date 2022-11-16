/**
 * \file dispatcher.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

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
#include "cosm/foraging/block_dist/dist_status.hpp"
#include "cosm/spatial/conflict_checker.hpp"
#include "cosm/foraging/block_dist/base_distributor.hpp"

#include "rcppsw/types/discretize_ratio.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/rng.hpp"
#include "rcppsw/math/vector3.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {
class multicell_entity;
} // namespace repr

namespace cosm::arena::ds {
class arena_grid;
} // namespace ds

NS_START(cosm, foraging, block_dist);

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
class dispatcher final : public rer::client<dispatcher> {
 public:
  static inline const std::string kDistSingleSrc = "single_source";
  static inline const std::string kDistRandom = "random";
  static inline const std::string kDistDualSrc = "dual_source";
  static inline const std::string kDistQuadSrc = "quad_source";
  static inline const std::string kDistPowerlaw = "powerlaw";

  dispatcher(cads::arena_grid* grid,
             const rtypes::discretize_ratio& resolution,
             const config::block_dist_config* config);
  ~dispatcher(void);

  dispatcher(const dispatcher&) = delete;
  dispatcher& operator=(const dispatcher&) = delete;
  const std::string& dist_type(void) { return mc_dist_type; }

  /**
   * \brief Initialize the selected block distributor. This is a separate
   * function, rather than happening in the constructor, so that error handling
   * can be done without exceptions.
   *
   * \param entities The entities that (might) need to be avoided when clusters
   *                 are initialized.
   * \param block_bb A bounding box large enough to hold any block in the
   *                 arena.
   * \return \c TRUE if initialization successful, \c FALSE otherwise.
   */
  bool initialize(carena::base_arena_map* map,
                  const cds::const_spatial_entity_vector& entities,
                  const rmath::vector3d& block_bb,
                  const cspatial::conflict_checker::map_cb_type& conflict_check,
                  const base_distributor::dist_success_cb_type& dist_success,
                  rmath::rng* rng);

  /**
   * \brief Distribute a block in the arena.
   *
   * \param block The block to distribute.
   *
   * \return \c TRUE iff distribution was successful, \c FALSE otherwise.
   */
  dist_status distribute_block(crepr::sim_block3D* block);

  /**
   * \brief Distribute all blocks in the arena.
   *
   * \return \c TRUE iff distribution was successful, \c FALSE otherwise.
   */
  dist_status distribute_blocks(cds::block3D_vectorno& blocks);

  const base_distributor* distributor(void) const {
    return m_dist.get();
  }
  base_distributor* distributor(void) { return m_dist.get(); }

  const rmath::rangez& distributable_cellsx(void) const { return mc_cells_xrange; }
  const rmath::rangez& distributable_cellsy(void) const { return mc_cells_yrange; }

 private:
  /* clang-format off */
  const rtypes::discretize_ratio                mc_resolution;
  const config::block_dist_config               mc_config;
  const std::string                             mc_dist_type;

  rmath::rangez                                 mc_cells_xrange;
  rmath::rangez                                 mc_cells_yrange;

  cads::arena_grid*                              m_grid{nullptr};
  std::unique_ptr<base_distributor>             m_dist;
  /* clang-format on */
};

NS_END(block_dist, foraging, cosm);

