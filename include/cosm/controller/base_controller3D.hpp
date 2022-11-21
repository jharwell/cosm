/**
 * \file base_controller3D.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
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
#include "cosm/subsystem/subsystem_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::controller {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_controller3D
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
class base_controller3D : public base_controller,
                           public csmetrics::dist3D_metrics {
 public:
  base_controller3D(void) RCPPSW_COLD;
  ~base_controller3D(void) override RCPPSW_COLD;

  base_controller3D(const base_controller3D&) = delete;
  base_controller3D& operator=(const base_controller3D&) = delete;

  /* swarm spatial distribution 3D metrics */
  rmath::vector3d rpos3D(void) const override final RCPPSW_PURE;
  rmath::vector3z dpos3D(void) const override final RCPPSW_PURE;
  rmath::radians azimuth(void) const override final RCPPSW_PURE;
  rmath::radians zenith(void) const override final RCPPSW_PURE;

  void sensing_update(const rtypes::timestep& tick,
                      const rtypes::discretize_ratio& ratio) override;
  void sensing_update(const rtypes::timestep& tick) override;

  void mdc_ts_update(void) const override final;
};

} /* namespace cosm::controller */
