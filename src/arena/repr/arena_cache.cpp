/**
 * \file arena_cache.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/arena/repr/arena_cache.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, arena, repr);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
arena_cache::arena_cache(const base_cache::params& p,
                         const rutils::color& light_color)
    : base_cache(p),
      m_light(new ::argos::CLightEntity(
          "cache_light" + rcppsw::to_string(id()),
          ::argos::CVector3(rcenter2D().x(), rcenter2D().y(), 0.0),
          ::argos::CColor(light_color.red(),
                          light_color.green(),
                          light_color.blue()),
          1.0)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void arena_cache::reset_metrics(void) {
  m_block_pickups = 0;
  m_block_drops = 0;
  m_penalty_count = rtypes::timestep(0);
} /* reset_metrics() */

NS_END(repr, arena, cosm);
