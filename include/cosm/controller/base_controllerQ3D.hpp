/**
 * \file base_controllerQ3D.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <string>
#include <typeindex>

#include "rcppsw/math/vector3.hpp"

#include "cosm/controller/base_controller.hpp"
#include "cosm/spatial/metrics/dist3D_metrics.hpp"
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
 * \class base_controllerQ3D
 * \ingroup controller
 *
 * \brief The implementation of base controller class that the controllers for
 * all *wheeled * (or otherwise mobile ground) robots that operate in 3D derive
 * (and therefore are "quasi" 3D). It uses a 2D actuation subsystem to move in
 * 3D through 2D actuation (e.g. driving up ramps). It uses a 3D sensing
 * subsystem to track position.
 *
 * This is NOT a controller for true 3D robots such as quadcopters that can both
 * sense and actuate in 3D.
 *
 * It should never be derived from directly; derive from one of the adaptor
 * controllers in the PAL.
 */
class base_controllerQ3D : public base_controller,
                           public csmetrics::movement_metrics,
                           public csmetrics::goal_acq_metrics,
                           public csmetrics::dist3D_metrics {
 public:
  base_controllerQ3D(void) RCPPSW_COLD;
  ~base_controllerQ3D(void) override RCPPSW_COLD;

  base_controllerQ3D(const base_controllerQ3D&) = delete;
  base_controllerQ3D& operator=(const base_controllerQ3D&) = delete;

  /* swarm spatial distribution 3D metrics */
  rmath::vector3d rpos3D(void) const override final RCPPSW_PURE;
  rmath::vector3z dpos3D(void) const override final RCPPSW_PURE;
  rmath::radians azimuth(void) const override final RCPPSW_PURE;
  rmath::radians zenith(void) const override final RCPPSW_PURE;

  void sensing_update(const rtypes::timestep& tick,
                      const rtypes::discretize_ratio& ratio) override;
  void sensing_update(const rtypes::timestep& tick) override;

  /**
   * \brief For less typing when doing operations with the arena map, which is
   * (logically) a 2D object.
   */
  rmath::vector2d rpos2D(void) const RCPPSW_PURE;
  rmath::vector2z dpos2D(void) const RCPPSW_PURE;

  /**
   * \brief Provided for compatibility with 2D metric gathering without having
   * to resort to template trickery.
   */
  rmath::radians heading2D(void) const RCPPSW_PURE;

  void mdc_ts_update(void) const override final;
  const class subsystem::saa_subsystemQ3D* saa(void) const { return m_saa.get(); }

 protected:
  class subsystem::saa_subsystemQ3D* saa(void) {
    return m_saa.get();
  }
  void saa(std::unique_ptr<subsystem::saa_subsystemQ3D> saa);

  rspatial::euclidean_dist ts_distance_impl(void) const RCPPSW_PURE;
  rmath::vector3d ts_velocity_impl(void) const;

 private:
  /* clang-format off */
  std::unique_ptr<subsystem::saa_subsystemQ3D> m_saa{nullptr};
  /* clang-format on */
};

NS_END(controller, cosm);
