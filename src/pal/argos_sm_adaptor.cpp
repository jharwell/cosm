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

#include "cosm/arena/config/arena_map_config.hpp"
#include "cosm/arena/base_arena_map.hpp"
#include "cosm/arena/caching_arena_map.hpp"
#include "cosm/vis/config/visualization_config.hpp"
#include "cosm/pal/embodied_block_creator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, pal);

void ___sighandler(int signum);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
argos_sm_adaptor::argos_sm_adaptor(void)
    : ER_CLIENT_INIT("cosm.pal.argos_sm_adaptor") {}

argos_sm_adaptor::~argos_sm_adaptor(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
template<typename TArenaMapType>
void argos_sm_adaptor::arena_map_init(
    const caconfig::arena_map_config* aconfig,
    const cvconfig::visualization_config* vconfig) {
  m_arena_map = std::make_unique<TArenaMapType>(aconfig);

  auto map = boost::get<std::unique_ptr<TArenaMapType>>(m_arena_map).get();
  if (!map->initialize(this, rng())) {
    ER_ERR("Could not initialize arena map");
    std::exit(EXIT_FAILURE);
  }

  map->distribute_all_blocks();

  /*
   * If null, visualization has been disabled.
   */
  if (nullptr != vconfig) {
    for (auto& block : map->blocks()) {
      block->vis_id(vconfig->block_id);
    } /* for(&block..) */
  }
} /* arena_map_init() */

crepr::embodied_block_variant argos_sm_adaptor::make_embodied(
    const crepr::block3D_variant& block,
    const rmath::radians& z_rotation,
    const rtypes::type_uuid& parent_id) {
  auto visitor = std::bind(embodied_block_creator(),
                           std::placeholders::_1,
                           z_rotation,
                           parent_id,
                           this);
  return boost::apply_visitor(visitor, block);
} /* make_embodied() */

/*******************************************************************************
 * Template Instantiations
 ******************************************************************************/
template void argos_sm_adaptor::arena_map_init<carena::base_arena_map<crepr::base_block2D>>(const caconfig::arena_map_config* aconfig,
                                                                                            const cvconfig::visualization_config* vconfig);
template void argos_sm_adaptor::arena_map_init<carena::base_arena_map<crepr::base_block3D>>(const caconfig::arena_map_config* aconfig,
                                                                                            const cvconfig::visualization_config* vconfig);
template void argos_sm_adaptor::arena_map_init<carena::caching_arena_map>(const caconfig::arena_map_config* aconfig,
                                                                          const cvconfig::visualization_config* vconfig);


NS_END(pal, cosm);
