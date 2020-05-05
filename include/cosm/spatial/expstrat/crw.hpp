/**
 * \file crw.hpp
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

#ifndef INCLUDE_COSM_SPATIAL_EXPSTRAT_CRW_HPP_
#define INCLUDE_COSM_SPATIAL_EXPSTRAT_CRW_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/rng.hpp"

#include "cosm/spatial/collision_tracker.hpp"
#include "cosm/spatial/expstrat/base_expstrat.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, spatial, expstrat);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class crw
 * \ingroup spatial expstrat
 *
 * \brief Roam around using Correlated Random Walk looking for something until
 * you happen to stumble across it.
 */
class crw : public csexpstrat::base_expstrat,
            public rer::client<crw> {
 public:
  crw(csubsystem::saa_subsystemQ3D* saa, rmath::rng* rng);

  ~crw(void) override = default;
  crw(const crw&) = delete;
  crw& operator=(const crw&) = delete;

  /* taskable overrides */
  void task_start(cta::taskable_argument*) override final {
    m_task_running = true;
  }
  void task_reset(void) override final { m_task_running = false; }
  bool task_running(void) const override final { return m_task_running; }

  /**
   * \brief Since we are exploring for something we don't know about, we will
   * never finish (stopping exploration is handled at a higher level).
   */
  bool task_finished(void) const override final { return false; }
  void task_execute(void) override final;

  /* prototype overrides */
  std::unique_ptr<base_expstrat> clone(void) const override {
    return std::make_unique<crw>(saa(), rng());
  }

 private:
  /* clang-format off */
  bool                        m_task_running{false};
  cspatial::collision_tracker m_tracker;
  /* clang-format on */

 public:
  /* collision metrics */
  RCPPSW_DECLDEF_WRAP_OVERRIDE(in_collision_avoidance, m_tracker, const)
  RCPPSW_DECLDEF_WRAP_OVERRIDE(entered_collision_avoidance, m_tracker, const)
  RCPPSW_DECLDEF_WRAP_OVERRIDE(exited_collision_avoidance, m_tracker, const)
  RCPPSW_DECLDEF_WRAP_OVERRIDE(collision_avoidance_duration, m_tracker, const)
  RCPPSW_DECLDEF_WRAP_OVERRIDE(avoidance_loc2D, m_tracker, const)
  RCPPSW_DECLDEF_WRAP_OVERRIDE(avoidance_loc3D, m_tracker, const)
};

NS_END(expstrat, spatial, cosm);

#endif /* INCLUDE_COSM_SPATIAL_EXPSTRAT_CRW_HPP_ */
