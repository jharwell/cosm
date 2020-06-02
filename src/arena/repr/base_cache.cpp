/**
 * \file base_cache.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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
#include "cosm/arena/repr/base_cache.hpp"

#include "cosm/repr/base_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, arena, repr);

/*******************************************************************************
 * Static Members
 ******************************************************************************/
int base_cache::m_next_id = 0;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
base_cache::base_cache(const params& p)
    : unicell_immovable_entity2D(rmath::vector2d(p.dimension.v(),
                                                 p.dimension.v()),
                                 p.center,
                                 p.resolution),
      colored_entity(rutils::color::kGRAY40),
      mc_resolution(p.resolution),
      m_blocks(p.blocks) {
  if (rtypes::constants::kNoUUID == p.id) {
    entity2D::id(rtypes::type_uuid(m_next_id++));
  } else {
    entity2D::id(rtypes::type_uuid(p.id));
  }
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void base_cache::block_remove(crepr::base_block3D* const block) {
  m_blocks.erase(std::find_if(m_blocks.begin(),
                              m_blocks.end(),
                              [&](const auto& b) { return b->idcmp(*block); }));
} /* block_remove() */

std::unique_ptr<base_cache> base_cache::clone(void) const {
  return std::make_unique<base_cache>(params{
      rtypes::spatial_dist(xdimr()), mc_resolution, rpos2D(), m_blocks, id()});
} /* clone() */

RCSW_PURE bool base_cache::contains_block(
    const crepr::base_block3D* const c_block) const {
  return std::find_if(m_blocks.begin(), m_blocks.end(), [&](const auto& b) {
           return b->id() == c_block->id();
         }) != m_blocks.end();
}
NS_END(repr, arena, cosm);
