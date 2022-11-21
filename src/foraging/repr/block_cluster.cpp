/**
 * \file block_cluster.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/foraging/repr/block_cluster.hpp"

#include "cosm/repr/sim_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::foraging::repr {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_cluster::blocks_recalc(void) {
  m_blocks.clear();
  for (size_t i = 0; i < xdsize(); ++i) {
    for (size_t j = 0; j < ydsize(); ++j) {
      auto& cell = access(i, j);
      /*
       * You can't assert that the cell does not have a cache/is not part of a
       * cache extent, because for RN distributions caches can be in the middle
       * of the main block cluster.
       */
      if (cell.state_has_block()) {
        ER_ASSERT(nullptr != cell.block3D(),
                  "Cell@%s null block3D",
                  rcppsw::to_string(cell.loc()).c_str());
        m_blocks.push_back(cell.block3D());
      }
    } /* for(j..) */
  } /* for(i..) */
} /* blocks_recalc() */

void block_cluster::update_after_drop(const crepr::sim_block3D* dropped) {
  ER_ASSERT(contains_abs(dropped->danchor2D()),
            "Block%s@%s not contained in cluster%s extent: xspan=%s,yspan=%s",
            rcppsw::to_string(dropped->id()).c_str(),
            rcppsw::to_string(dropped->danchor2D()).c_str(),
            rcppsw::to_string(this->id()).c_str(),
            rcppsw::to_string(xdspan()).c_str(),
            rcppsw::to_string(ydspan()).c_str());

  auto relative_to = dropped->danchor2D() - danchor2D();

  auto& cell = access(relative_to);

  ER_ASSERT(cell.state_has_block(),
            "Cell@%s not in HAS_BLOCK state: state=%d",
            rcppsw::to_string(dropped->danchor2D()).c_str(),
            cell.fsm().current_state());
  ER_ASSERT(dropped->id() == cell.block3D()->id(),
            "Cell@%s block%s != dropped block%s",
            rcppsw::to_string(cell.loc()).c_str(),
            rcppsw::to_string(cell.block3D()->id()).c_str(),
            rcppsw::to_string(dropped->id()).c_str());

  ER_DEBUG("Add dropped block%d@%s/%s to cluster%d w/xspan=%s,yspan=%s",
           dropped->id().v(),
           rcppsw::to_string(dropped->ranchor2D()).c_str(),
           rcppsw::to_string(dropped->danchor2D()).c_str(),
           this->id().v(),
           rcppsw::to_string(xdspan()).c_str(),
           rcppsw::to_string(ydspan()).c_str());
  m_blocks.push_back(dropped);
} /* update_after_drop() */

void block_cluster::update_after_pickup(const rtypes::type_uuid& pickup_id) {
  auto it = std::find_if(m_blocks.begin(), m_blocks.end(), [&](const auto* b) {
    return b->id() == pickup_id;
  });
  ER_ASSERT(it != m_blocks.end(),
            "Block%s not in cluster%s",
            rcppsw::to_string(pickup_id).c_str(),
            rcppsw::to_string(this->id()).c_str());

  ER_DEBUG("Remove block%d from cluster%d w/xspan=%s,yspan=%s",
           pickup_id.v(),
           this->id().v(),
           rcppsw::to_string(xdspan()).c_str(),
           rcppsw::to_string(ydspan()).c_str());
  m_blocks.erase(std::remove(m_blocks.begin(), m_blocks.end(), *it));

  it = std::find_if(m_blocks.begin(), m_blocks.end(), [&](const auto* b) {
    return b->id() == pickup_id;
  });
  ER_ASSERT(it == m_blocks.end(),
            "Block%s still in cluster%s",
            rcppsw::to_string(pickup_id).c_str(),
            rcppsw::to_string(this->id()).c_str());
} /* update_after_pickup() */

} /* namespace cosm::foraging::repr */
