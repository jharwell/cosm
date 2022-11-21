/**
 * \file base_controller2D.hpp
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
#include <vector>

#include "rcppsw/math/vector2.hpp"

#include "cosm/controller/base_controller.hpp"
#include "cosm/spatial/metrics/dist2D_metrics.hpp"
#include "cosm/subsystem/subsystem_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::tv {
class irv_manager;
} /* namespace cosm::tv */

namespace cosm::controller {

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

  void sensing_update(const rtypes::timestep& tick) override;

  void mdc_ts_update(void) const override final;
};

} /* namespace cosm::controller */
