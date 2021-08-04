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
      mc_resolution(p.resolution),
      m_blocks_vec(std::move(p.blocks)) {
  /* /\* build the block map *\/ */
  /* std::transform(p.blocks.begin(), */
  /*                p.blocks.end(), */
  /*                std::inserter(m_blocks, m_blocks.end()), */
  /*                [&](auto* b) { return std::make_pair(b->id(), b); }); */
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void base_cache::block_add(crepr::base_block3D* block) {
  m_blocks_vec.push_back(block);
  if (m_map_en) {
    m_blocks_map.insert(std::make_pair(block->id(), block));
  }
}
void base_cache::block_remove(const crepr::base_block3D* const victim) {
  if (m_map_en) {
    m_blocks_map.erase(m_blocks_map.find(victim->id()));
  }
  auto it = std::find_if(m_blocks_vec.begin(),
                         m_blocks_vec.end(),
                         [&](const auto*b) { return b->id() == victim->id(); });
  m_blocks_vec.erase(it);
} /* block_remove() */

void base_cache::blocks_map_enable(void) {
  for (auto *b : m_blocks_vec) {
    m_blocks_map.insert(std::make_pair(b->id(), b));
  } /* for(*b..) */

  m_map_en = true;
} /* blocks_map_enable() */

std::unique_ptr<base_cache> base_cache::clone(void) const {
  /* cds::block3D_vectorno blocks; */
  cds::block3D_vectorno for_cache = m_blocks_vec;
  params p = { xrsize(), mc_resolution, rcenter2D(), std::move(for_cache), id() };
  return std::make_unique<base_cache>(p);
} /* clone() */

bool base_cache::contains_block(const crepr::base_block3D* const c_block) const {
  if (m_map_en) {
    return m_blocks_map.end() != m_blocks_map.find(c_block->id());
  } else {
    return m_blocks_vec.end() != std::find_if(m_blocks_vec.begin(),
                                              m_blocks_vec.end(),
                                              [&](const auto*b) { return b->id() == c_block->id(); });
  }
}
crepr::base_block3D* base_cache::block_select(rmath::rng* rng) {
  ER_ASSERT(m_blocks_vec.size() > 0, "Cannot select from empty block vector");
  if (nullptr != rng) {
    size_t dist = rng->uniform(0UL, m_blocks_vec.size() - 1);
    return m_blocks_vec[dist];
    /* auto it = m_blocks_vec.begin(); */
    /* std::advance(it, dist); */
    /* return it->second; */
  } else {
    return m_blocks_vec[0];
  }
}
NS_END(repr, arena, cosm);
