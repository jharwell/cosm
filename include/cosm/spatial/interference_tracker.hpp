/**
 * \file interference_tracker.hpp
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

#ifndef INCLUDE_COSM_SPATIAL_INTERFERENCE_TRACKER_HPP_
#define INCLUDE_COSM_SPATIAL_INTERFERENCE_TRACKER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/cosm.hpp"
#include "cosm/spatial/metrics/interference_metrics.hpp"
#include "cosm/subsystem/subsystem_fwd.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class interference_tracker
 * \ingroup spatial
 *
 * \brief Utility class for tracking when a robot enters/exits a interference
 * avoidance state, and the time spent in that state, as it moves in 2D or 3D.
 */
class interference_tracker : public csmetrics::interference_metrics {
 public:
  explicit interference_tracker(
      const csubsystem::sensing_subsystemQ3D* const sensing)
      : mc_sensing(sensing) {}

  interference_tracker(const interference_tracker&) = delete;
  interference_tracker& operator=(const interference_tracker&) = delete;

  /* interference metrics */
  bool exp_interference(void) const override final RCPPSW_PURE;
  bool entered_interference(void) const override final RCPPSW_PURE;
  bool exited_interference(void) const override final RCPPSW_PURE;
  rtypes::timestep interference_duration(void) const override final;
  rmath::vector3z interference_loc3D(void) const override final RCPPSW_PURE;

  /**
   * \brief Handle all logic for entering interference avoidance; classes should
   * only have to call this function whenever they detect an obstacle.
   */
  void inta_enter(void);

  /**
   * \brief Handle all logic for exiting interference avoidance; classes should
   * only have to call this function whenever they no longer detect any
   * obstacles.
   */
  void inta_exit(void);

  void inta_reset(void);

 private:
  /* clang-format off */
  const subsystem::sensing_subsystemQ3D* const mc_sensing;

  bool                                      m_entered_interference{false};
  bool                                      m_exited_interference{false};
  bool                                      m_exp_interference{false};
  rtypes::timestep                          m_interference_start{0};
  /* clang-format on */
};

NS_END(spatial, cosm);

#endif /* INCLUDE_COSM_SPATIAL_INTERFERENCE_TRACKER_HPP_ */
