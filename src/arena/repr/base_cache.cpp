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
    : unicell_immovable_entity2D(
          rtypes::constants::kNoUUID == p.id ? rtypes::type_uuid(m_next_id++)
                                             : p.id,
          rmath::vector2d(p.dimension.v(), p.dimension.v()),
          p.resolution,
          p.center),
      ER_CLIENT_INIT("cosm.arena.repr.base_cache"),
      colored_entity(rutils::color::kGRAY40),
      mc_resolution(p.resolution) {
  /* build the block map */
  std::transform(p.blocks.begin(),
                 p.blocks.end(),
                 std::inserter(m_blocks, m_blocks.end()),
                 [&](auto* b) { return std::make_pair(b->id(), b); });
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void base_cache::block_add(crepr::base_block3D* block) {
  m_blocks.insert({ block->id(), block });
}

void base_cache::block_remove(const crepr::base_block3D* const victim) {
  m_blocks.erase(m_blocks.find(victim->id()));
} /* block_remove() */

std::unique_ptr<base_cache> base_cache::clone(void) const {
  cds::block3D_vectorno blocks;
  params p = { xrsize(), mc_resolution, rcenter2D(), blocks, id() };
  std::transform(m_blocks.begin(),
                 m_blocks.end(),
                 std::back_inserter(p.blocks),
                 [&](auto& pair) { return pair.second; });
  return std::make_unique<base_cache>(p);
} /* clone() */

bool base_cache::contains_block(const crepr::base_block3D* const c_block) const {
  return m_blocks.end() != m_blocks.find(c_block->id());
}
crepr::base_block3D* base_cache::block_select(rmath::rng* rng) {
  ER_ASSERT(m_blocks.size() > 0, "Cannot select from empty block hashtable");
  if (nullptr != rng) {
    size_t dist = rng->uniform(0UL, m_blocks.size() - 1);
    auto it = m_blocks.begin();
    std::advance(it, dist);
    return it->second;
  } else {
    return m_blocks.begin()->second;
  }
}
NS_END(repr, arena, cosm);
