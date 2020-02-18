/**
 * \file arena_cache.hpp
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

#ifndef INCLUDE_COSM_FORAGING_REPR_ARENA_CACHE_HPP_
#define INCLUDE_COSM_FORAGING_REPR_ARENA_CACHE_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <algorithm>
#include <vector>

#include <argos3/plugins/simulator/entities/light_entity.h>

#include "cosm/foraging/metrics/caches/location_metrics.hpp"
#include "cosm/foraging/metrics/caches/utilization_metrics.hpp"
#include "cosm/foraging/repr/base_cache.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class arena_cache
 * \ingroup foraging repr
 *
 * \brief A representation of an ACTUAL cache within the arena (as opposed to a
 * robot's representation of a cache, which may not be accuratef).
 *
 * Arena caches:
 *
 * - Handle cache penalties
 * - Can collect metrics about their usage
 * - Have a light/beacon at ground level robots can see.
 */
class arena_cache final : public base_cache,
                          public metrics::caches::utilization_metrics,
                          public metrics::caches::location_metrics {
 public:
  arena_cache(const base_cache::params& p, const rutils::color& light_color);
  ~arena_cache(void) override = default;

  arena_cache(const arena_cache&) = delete;
  arena_cache& operator=(const arena_cache&) = delete;

  /* metrics */
  size_t n_blocks(void) const override { return base_cache::n_blocks(); }
  uint total_block_pickups(void) const override { return m_block_pickups; }
  uint total_block_drops(void) const override { return m_block_drops; }
  void reset_metrics(void) override;
  rmath::vector2u location(void) const override { return dloc(); }

  /**
   * \brief Set a flag indicating the cache has had a block pickup this
   * timestep.
   */
  void has_block_pickup(void) { m_block_pickups = 1; }

  /**
   * \brief Set a flag indicating the cache has had a block drop this timestep.
   */
  void has_block_drop(void) { m_block_drops = 1; }

  /**
   * \brief Increment the cumulative count of penalties served for both pickups
   * and drops. Reset by \ref reset_metrics().
   */
  void penalty_served(const rtypes::timestep& duration) {
    m_penalty_count += duration;
  }

  argos::CLightEntity* light(void) const { return m_light; }

 private:
  /* clang-format off */
  uint                 m_block_pickups{0};
  uint                 m_block_drops{0};
  rtypes::timestep     m_penalty_count{0};
  argos::CLightEntity* m_light;
  /* clang-format on */
};

NS_END(repr, foraging, cosm);

#endif /* INCLUDE_COSM_FORAGING_REPR_ARENA_CACHE_HPP_ */
