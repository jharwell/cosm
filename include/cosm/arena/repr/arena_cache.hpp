/**
 * \file arena_cache.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <algorithm>
#include <vector>

#include <argos3/plugins/simulator/entities/light_entity.h>

#include "cosm/arena/metrics/caches/location_metrics.hpp"
#include "cosm/arena/metrics/caches/utilization_metrics.hpp"
#include "cosm/arena/repr/base_cache.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena::repr {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class arena_cache
 * \ingroup arena repr
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
  size_t total_block_pickups(void) const override { return m_block_pickups; }
  size_t total_block_drops(void) const override { return m_block_drops; }
  void reset_metrics(void) override;
  rmath::vector2z location(void) const override { return dcenter2D(); }

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

  ::argos::CLightEntity* light(void) const { return m_light; }

 private:
  /* clang-format off */
  size_t                 m_block_pickups{0};
  size_t                 m_block_drops{0};
  rtypes::timestep       m_penalty_count{0};
  ::argos::CLightEntity* m_light;
  /* clang-format on */
};

} /* namespace cosm::arena::repr */

