/**
 * \file oracle_manager.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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
#include "cosm/oracle/oracle_manager.hpp"

#include "cosm/foraging/ds/arena_map.hpp"
#include "cosm/oracle/config/oracle_manager_config.hpp"
#include "cosm/oracle/entities_oracle.hpp"
#include "cosm/oracle/tasking_oracle.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, oracle);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
oracle_manager::oracle_manager(const coconfig::oracle_manager_config* const config)
    : m_entities(std::make_unique<class entities_oracle>(&config->entities)),
      m_tasking(nullptr) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void oracle_manager::tasking_oracle(std::unique_ptr<class tasking_oracle> o) {
  m_tasking = std::move(o);
} /* tasking_oracle */

void oracle_manager::update(cfds::arena_map* const map) {
  if (m_entities->blocks_enabled()) {
    entities_oracle::variant_vector_type v;
    /*
     * Updates to oracle manager can happen in parallel, so we want to make sure
     * we don't get a set of blocks in a partially updated state. See #594.
     */
    std::scoped_lock lock(*map->block_mtx());

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
    m_entities->set_blocks(v);
  }

  if (m_entities->caches_enabled()) {
    std::scoped_lock lock(*map->cache_mtx());
    entities_oracle::variant_vector_type v;
    for (auto& c : map->caches()) {
      v.push_back(c);
    } /* for(&b..) */
    m_entities->set_caches(v);
  }
} /* update() */

NS_END(oracle, cosm);
