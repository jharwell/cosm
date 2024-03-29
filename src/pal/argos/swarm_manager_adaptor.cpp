/**
 * \file swarm_manager_adaptor.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/pal/argos/swarm_manager_adaptor.hpp"

#include "rcppsw/math/rngm.hpp"

#include "cosm/arena/base_arena_map.hpp"
#include "cosm/arena/caching_arena_map.hpp"
#include "cosm/arena/config/arena_map_config.hpp"
#include "cosm/argos/block_embodiment_creator.hpp"
#include "cosm/argos/vis/config/visualization_config.hpp"
#include "cosm/repr/sim_block3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::pal::argos {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
swarm_manager_adaptor::swarm_manager_adaptor(void)
    : ER_CLIENT_INIT("cosm.pal.argos.swarm_manager_adaptor") {}

swarm_manager_adaptor::~swarm_manager_adaptor(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
template <typename TArenaMap>
void swarm_manager_adaptor::arena_map_create(
    const caconfig::arena_map_config* aconfig) {
  m_arena_map = std::make_unique<TArenaMap>(aconfig, rng());
} /* arena_map_create() */

void swarm_manager_adaptor::arena_map_init(
    const cavis::config::visualization_config* vconfig,
    const crepr::config::nests_config* nconfig) {
  /*
   * If null, visualization has been disabled.
   */
  if (nullptr != vconfig) {
    for (auto& block : m_arena_map->blocks()) {
      block->vis_id(vconfig->block_id);
    } /* for(&block..) */
  }

  if (!m_arena_map->initialize(this, nconfig)) {
    ER_ERR("Could not initialize arena map");
    std::exit(EXIT_FAILURE);
  }
} /* arena_map_init() */

::argos::CColor
swarm_manager_adaptor::GetFloorColor(const ::argos::CVector2& pos) {
  /* no arena map -> can't change the color */
  if (nullptr == m_arena_map) {
    return ::argos::CColor::WHITE;
  }

  rmath::vector2d rpos(pos.GetX(), pos.GetY());
  rmath::vector2z dpos =
      rmath::dvec2zvec(rpos, m_arena_map->grid_resolution().v());

  /*
   * The point we are handed can be out of bounds of the discretized extent of
   * the arena, depending on the size of the robot. For smaller robots which can
   * get their center closer to the edge of the arena, if their ground sensors
   * are out on the front (e.g., ARGoS e-puck), then the point we are passed can
   * be out of bounds. In such cases, always return white.
   *
   * See COSM#7.
   */
  if (dpos.x() < m_arena_map->xdsize() && dpos.y() < m_arena_map->xdsize()) {
    auto color = m_arena_map->access<cads::arena_grid::kCell>(dpos).color();
    return ::argos::CColor(color.red(), color.green(), color.blue());
  } else {
    return ::argos::CColor::WHITE;
  }
} /* GetFloorColor() */

/*******************************************************************************
 * ARGoS Hooks
 ******************************************************************************/
void swarm_manager_adaptor::init(ticpp::Element&) {
  m_floor = &GetSpace().GetFloorEntity();
} /* Init() */

void swarm_manager_adaptor::pre_step(void) {
  timestep(rtypes::timestep(GetSpace().GetSimulationClock()));
} /* pre_step() */

/*******************************************************************************
 * Template Instantiations
 ******************************************************************************/
template void swarm_manager_adaptor::arena_map_create<carena::base_arena_map>(
    const caconfig::arena_map_config* aconfig);
template void swarm_manager_adaptor::arena_map_create<carena::caching_arena_map>(
    const caconfig::arena_map_config* aconfig);

} /* namespace cosm::pal::argos */
