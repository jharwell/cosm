/**
 * \file foraging_oracle.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/foraging/oracle/foraging_oracle.hpp"

#include "cosm/arena/base_arena_map.hpp"
#include "cosm/arena/caching_arena_map.hpp"
#include "cosm/oracle/config/aggregate_oracle_config.hpp"
#include "cosm/oracle/entities_oracle.hpp"
#include "cosm/oracle/tasking_oracle.hpp"
#include "cosm/repr/sim_block3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::foraging::oracle {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
foraging_oracle::foraging_oracle(const coconfig::aggregate_oracle_config* config)
    : aggregate_oracle(config) {
  oracle_add(kBlocks,
             std::make_unique<coracle::entities_oracle<crepr::sim_block3D>>());
  oracle_add(kCaches,
             std::make_unique<coracle::entities_oracle<carepr::base_cache>>());
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void foraging_oracle::tasking_oracle(std::unique_ptr<coracle::tasking_oracle> o) {
  oracle_add(kTasks, std::move(o));
} /* tasking_oracle */

void foraging_oracle::update(carena::base_arena_map* const map) {
  auto blocks_it = config()->entities.types.find("blocks");
  if (config()->entities.types.end() != blocks_it && blocks_it->second) {
    coracle::entities_oracle<crepr::sim_block3D>::knowledge_type v;
    /*
     * Updates to oracle manager can happen in parallel, so we want to make sure
     * we don't get a set of blocks in a partially updated state. See #594.
     */
    std::shared_lock lock(*map->block_mtx());

    std::copy_if(map->blocks().begin(),
                 map->blocks().end(),
                 std::back_inserter(v),
                 [&](const auto& b) {
                   /* don't include blocks robot's are carrying */
                   return rtypes::constants::kNoUUID == b->md()->robot_id();
                 });
    oracle_get<coracle::entities_oracle<crepr::sim_block3D>>(kBlocks)
        ->set_knowledge(v);
  }
} /* update() */

void foraging_oracle::update(carena::caching_arena_map* const map) {
  auto blocks_it = config()->entities.types.find("blocks");
  if (config()->entities.types.end() != blocks_it && blocks_it->second) {
    coracle::entities_oracle<crepr::sim_block3D>::knowledge_type v;
    /*
     * Updates to oracle manager can happen in parallel, so we want to make sure
     * we don't get a set of blocks in a partially updated state. See #594.
     */
    std::shared_lock lock(*map->block_mtx());

    std::copy_if(map->blocks().begin(),
                 map->blocks().end(),
                 std::back_inserter(v),
                 [&](const auto& b) {
                   /* don't include blocks robot's are carrying */
                   return rtypes::constants::kNoUUID == b->md()->robot_id() &&
                          /*
                        * Don't include blocks that are currently in a cache
                        * (harmless, but causes repeated "removed block hidden
                        * behind cache" warnings)
                        */
                          std::none_of(map->caches().begin(),
                                       map->caches().end(),
                                       [&](const auto& c) {
                                         return c->contains_block(b);
                                       });
                 });
    oracle_get<coracle::entities_oracle<crepr::sim_block3D>>(kBlocks)
        ->set_knowledge(v);
  }

  auto caches_it = config()->entities.types.find("caches");
  if (config()->entities.types.end() != caches_it && caches_it->second) {
    std::shared_lock lock(*map->cache_mtx());
    coracle::entities_oracle<carepr::base_cache>::knowledge_type v;
    for (auto& c : map->caches()) {
      v.push_back(c);
    } /* for(&b..) */
    oracle_get<coracle::entities_oracle<carepr::base_cache>>(kCaches)
        ->set_knowledge(v);
  }
} /* update() */

} /* namespace cosm::foraging::oracle */
