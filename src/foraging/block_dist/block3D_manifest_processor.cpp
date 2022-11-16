/**
 * \file block3D_manifest_processor.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/foraging/block_dist/block3D_manifest_processor.hpp"

#include "cosm/repr/cube_block3D.hpp"
#include "cosm/repr/ramp_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, block_dist);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block3D_manifest_processor::block3D_manifest_processor(
    const config::block_manifest* const m,
    const rtypes::discretize_ratio& arena_res)
    : mc_arena_res(arena_res), mc_manifest(*m) {
  register_type<crepr::cube_block3D>("cube3D");
  register_type<crepr::ramp_block3D>("ramp3D");
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
cds::block3D_vectoro block3D_manifest_processor::operator()(void) {
  size_t i;
  cds::block3D_vectoro v;

  for (i = 0; i < mc_manifest.n_cube; ++i) {
    auto block = create("cube3D",
                        rtypes::type_uuid(i),
                        rmath::vector3d(mc_manifest.unit_dim,
                                        mc_manifest.unit_dim,
                                        mc_manifest.unit_dim),
                        mc_arena_res);
    /*
     * Move the block out of sight, so that if there are more blocks in the */
    /* arena map than can be successfully distributed, we don't run into weird */
    /* cases where the block has an undefined location.
     */
    block->move_out_of_sight();
    v.push_back(std::move(block));
  } /* for(i..) */
  for (i = mc_manifest.n_cube; i < mc_manifest.n_cube + mc_manifest.n_ramp; ++i) {
    auto block = create("ramp3D",
                        rtypes::type_uuid(i),
                        rmath::vector3d(mc_manifest.unit_dim * 2,
                                        mc_manifest.unit_dim,
                                        mc_manifest.unit_dim),
                        mc_arena_res);
    /*
     * Move the block out of sight, so that if there are more blocks in the
     * arena map than can be successfully distributed, we don't run into weird
     * cases where the block has an undefined location.
     */
    block->move_out_of_sight();
    v.push_back(std::move(block));
  } /* for(i..) */
  return v;
} /* operator()() */

NS_END(block_dist, foraging, cosm);
