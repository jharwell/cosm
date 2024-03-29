/**
 * \file base_cache.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/arena/repr/base_cache.hpp"

#include "cosm/repr/sim_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena::repr {

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
          /*
           * We add a TINY amount here to offset any floating point
           * representation errors which can arise from the subtraction (even
           * though we already checked and made sure the dimension would result
           * in an odd dsize cache, we still need to do this).
           */
          p.center - rmath::vector2d(p.dimension.v(), p.dimension.v()) / 2.0 +
              rmath::vector2d(rmath::kDOUBLE_EPSILON, rmath::kDOUBLE_EPSILON),
          p.resolution),
      ER_CLIENT_INIT("cosm.arena.repr.base_cache"),
      colored_entity(rutils::color::kGRAY40),
      mc_resolution(p.resolution),
      m_blocks_vec(p.blocks) {
  auto dims = rmath::vector2d(p.dimension.v(), p.dimension.v());
  auto anchor = p.center - dims / 2.0;
  ER_DEBUG("Configured center=%s/%s,anchor=%s/%s dims=%s",
           rcppsw::to_string(p.center).c_str(),
           rcppsw::to_string(p.center).c_str(),
           rcppsw::to_string(anchor).c_str(),
           rcppsw::to_string(rmath::dvec2zvec(anchor, p.resolution.v())).c_str(),
           rcppsw::to_string(dims).c_str());
  ER_DEBUG("Actual center=%s/%s,dims=%s/%s",
           rcppsw::to_string(rcenter2D()).c_str(),
           rcppsw::to_string(dcenter2D()).c_str(),
           rcppsw::to_string(rdims2D()).c_str(),
           rcppsw::to_string(ddims2D()).c_str());
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void base_cache::block_add(crepr::sim_block3D* block) {
  m_blocks_vec.push_back(block);
  if (m_map_en) {
    m_blocks_map.insert(std::make_pair(block->id(), block));
  }
}
void base_cache::block_remove(const crepr::sim_block3D* const victim) {
  if (m_map_en) {
    m_blocks_map.erase(m_blocks_map.find(victim->id()));
  }
  auto it = std::find_if(m_blocks_vec.begin(),
                         m_blocks_vec.end(),
                         [&](const auto* b) { return b->id() == victim->id(); });
  m_blocks_vec.erase(it);
} /* block_remove() */

void base_cache::blocks_map_enable(void) {
  for (auto* b : m_blocks_vec) {
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

bool base_cache::contains_block(const crepr::sim_block3D* const c_block) const {
  if (m_map_en) {
    return m_blocks_map.end() != m_blocks_map.find(c_block->id());
  } else {
    return m_blocks_vec.end() !=
           std::find_if(m_blocks_vec.begin(),
                        m_blocks_vec.end(),
                        [&](const auto* b) { return b->id() == c_block->id(); });
  }
}
crepr::sim_block3D* base_cache::block_select(rmath::rng* rng) {
  ER_ASSERT(m_blocks_vec.size() > 0, "Cannot select from empty block vector");
  if (nullptr != rng) {
    size_t dist = rng->uniform(rmath::rangez(0, m_blocks_vec.size() - 1));
    return m_blocks_vec[dist];
    /* auto it = m_blocks_vec.begin(); */
    /* std::advance(it, dist); */
    /* return it->second; */
  } else {
    return m_blocks_vec[0];
  }
}
} /* namespace cosm::arena::repr */
