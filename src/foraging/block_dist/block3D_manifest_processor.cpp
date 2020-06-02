/**
 * \file block3D_manifest_processor.cpp
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
    const config::block_manifest* const m)
    : mc_manifest(*m) {
  register_type<crepr::cube_block3D>("cube3D");
  register_type<crepr::ramp_block3D>("ramp3D");
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
cds::block3D_vectoro block3D_manifest_processor::operator()(void) {
  cds::block3D_vectoro v;
  uint i;
  for (i = 0; i < mc_manifest.n_cube; ++i) {
    v.push_back(create("cube3D",
                       rmath::vector3d(mc_manifest.unit_dim,
                                       mc_manifest.unit_dim,
                                       mc_manifest.unit_dim),
                       rtypes::type_uuid(i)));
  } /* for(i..) */
  for (i = mc_manifest.n_cube; i < mc_manifest.n_cube + mc_manifest.n_ramp;
       ++i) {
    v.push_back(create("ramp3D",
                       rmath::vector3d(mc_manifest.unit_dim * 2,
                                       mc_manifest.unit_dim,
                                       mc_manifest.unit_dim),
                       rtypes::type_uuid(i)));
  } /* for(i..) */
  return v;
} /* operator()() */

NS_END(block_dist, foraging, cosm);
