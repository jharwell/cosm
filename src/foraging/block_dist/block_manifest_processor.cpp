/**
 * \file block_manifest_processor.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/foraging/block_dist/block_manifest_processor.hpp"

#include "cosm/repr/cube_block2D.hpp"
#include "cosm/repr/ramp_block2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, block_dist);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_manifest_processor::block_manifest_processor(
    const config::block_manifest* const m)
    : mc_manifest(*m) {
  register_type<crepr::cube_block2D>("cube");
  register_type<crepr::ramp_block2D>("ramp");
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
ds::block2D_vectoro block_manifest_processor::create_blocks(void) {
  ds::block2D_vectoro v;
  uint i;
  for (i = 0; i < mc_manifest.n_cube; ++i) {
    v.push_back(
        create("cube",
               rmath::vector2d(mc_manifest.unit_dim, mc_manifest.unit_dim),
               rtypes::type_uuid(i)));
  } /* for(i..) */
  for (i = mc_manifest.n_cube; i < mc_manifest.n_cube + mc_manifest.n_ramp;
       ++i) {
    v.push_back(
        create("ramp",
               rmath::vector2d(mc_manifest.unit_dim * 2, mc_manifest.unit_dim),
               rtypes::type_uuid(i)));
  } /* for(i..) */
  return v;
} /* create_blocks() */

NS_END(block_dist, foraging, cosm);
