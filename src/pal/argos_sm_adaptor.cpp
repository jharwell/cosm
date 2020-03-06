/**
 * \file argos_sm_adaptor.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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
#include "cosm/pal/argos_sm_adaptor.hpp"

#include <sys/resource.h>

#include "rcppsw/math/rngm.hpp"
#include "cosm/foraging/config/arena_map_config.hpp"
#include "cosm/foraging/ds/arena_map.hpp"
#include "cosm/vis/config/visualization_config.hpp"
#include "cosm/oracle/oracle_manager.hpp"
#include "cosm/oracle/tasking_oracle.hpp"
#include "cosm/oracle/entities_oracle.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, pal);

void ___sighandler(int signum);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
argos_sm_adaptor::argos_sm_adaptor(void) :
    ER_CLIENT_INIT("cosm.pal.argos_sm_adaptor"),
    m_arena_map(nullptr),
    m_oracle_manager(nullptr) {}

argos_sm_adaptor::~argos_sm_adaptor(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void argos_sm_adaptor::arena_map_init(
    const cfconfig::arena_map_config * aconfig,
    const cvconfig::visualization_config* vconfig) {

  m_arena_map = std::make_unique<cfds::arena_map>(aconfig);

  if (!m_arena_map->initialize(this, rng())) {
    ER_ERR("Could not initialize arena map");
    std::exit(EXIT_FAILURE);
  }

  m_arena_map->distribute_all_blocks();

  /*
   * If null, visualization has been disabled.
   */
  if (nullptr != vconfig) {
    for (auto& block : m_arena_map->blocks()) {
      block->vis_id(vconfig->block_id);
    } /* for(&block..) */
  }
} /* arena_map_init() */

void argos_sm_adaptor::oracle_init(
    const coconfig::oracle_manager_config* const oraclep) {
  if (nullptr != oraclep) {
    ER_INFO("Creating oracle manager");
    m_oracle_manager = std::make_unique<coracle::oracle_manager>(oraclep);
  }
} /* oracle_init() */

NS_END(pal, cosm);
