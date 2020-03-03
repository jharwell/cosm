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
#include "cosm/foraging/repr/base_cache.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, repr);

/*******************************************************************************
 * Static Members
 ******************************************************************************/
int base_cache::m_next_id = 0;
constexpr size_t base_cache::kMinBlocks;

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
void base_cache::block_remove(crepr::base_block2D* const block) {
  m_blocks.erase(std::find_if(m_blocks.begin(),
                              m_blocks.end(),
                              [&](const auto& b) { return b->idcmp(*block); }));
} /* block_remove() */

std::unique_ptr<base_cache> base_cache::clone(void) const {
  return std::make_unique<base_cache>(params{
      rtypes::spatial_dist(xdimr()), mc_resolution, rloc(), blocks(), id()});
} /* clone() */

NS_END(repr, foraging, cosm);
