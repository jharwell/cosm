/**
 * \file base_controller2D.hpp
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

#ifndef INCLUDE_COSM_CONTROLLER_BASE_CONTROLLER2D_HPP_
#define INCLUDE_COSM_CONTROLLER_BASE_CONTROLLER2D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <ext/ticpp/ticpp.h>

#include <memory>
#include <string>
#include <typeindex>

#include "rcppsw/math/vector2.hpp"

#include "cosm/controller/base_controller.hpp"
#include "cosm/spatial/metrics/dist2D_metrics.hpp"
#include "cosm/spatial/metrics/goal_acq_metrics.hpp"
#include "cosm/spatial/metrics/movement_metrics.hpp"
#include "cosm/subsystem/subsystem_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::tv {
class irv_manager;
} /* namespace cosm::tv */

NS_START(cosm, controller);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_controller2D
 * \ingroup controller
 *
 * \brief The implementation of base controller class that the controllers for
 * all robots that operate in 2D derive.
 *
 * It should never be derived from directly; derive from one of the adaptor
 * controllers in the PAL.
 */
class base_controller2D : public base_controller,
                          public csmetrics::movement_metrics,
                          public csmetrics::goal_acq_metrics,
                          public csmetrics::dist2D_metrics {
 public:
  base_controller2D(void) RCPPSW_COLD;
  ~base_controller2D(void) override RCPPSW_COLD;

  base_controller2D(const base_controller2D&) = delete;
  base_controller2D& operator=(const base_controller2D&) = delete;

  /* swarm spatial distribution 2D metrics */
  rmath::vector2d rpos2D(void) const override final RCPPSW_PURE;
  rmath::vector2z dpos2D(void) const override final RCPPSW_PURE;
  rmath::radians heading2D(void) const override final RCPPSW_PURE;

  void sensing_update(const rtypes::timestep& tick,
                      const rtypes::discretize_ratio& ratio) override;

#if (LIBRA_ER >= LIBRA_ER_ALL)
  /**
   * \brief Convenience function to add robot ID+timestep to messages during
   * the control step.
   */
  void ndc_pusht(void) const override final;
#else
  void ndc_pusht(void) const override final {}
#endif

 protected:
  class subsystem::saa_subsystemQ3D* saa(void) {
    return m_saa.get();
  }
  const class subsystem::saa_subsystemQ3D* saa(void) const { return m_saa.get(); }
  void saa(std::unique_ptr<subsystem::saa_subsystemQ3D> saa);

  rtypes::spatial_dist ts_distance_impl(void) const RCPPSW_PURE;
  rmath::vector3d ts_velocity_impl(void) const;

 private:
  /* clang-format off */
  std::unique_ptr<subsystem::saa_subsystemQ3D>    m_saa;
  /* clang-format on */
};

NS_END(controller, cosm);

#endif /* INCLUDE_COSM_CONTROLLER_BASE_CONTROLLER2D_HPP_ */
