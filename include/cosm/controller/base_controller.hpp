/**
 * \file base_controller.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <ticpp/ticpp.h>

#include <memory>
#include <string>
#include <typeindex>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/rng.hpp"
#include "rcppsw/types/discretize_ratio.hpp"
#include "rcppsw/types/timestep.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "cosm/cosm.hpp"
#include "cosm/pal/config/output_config.hpp"
#include "cosm/pal/pal.hpp"
#include "cosm/spatial/common/interference_tracker.hpp"
#include "cosm/spatial/metrics/interference_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::fsm {
class supervisor_fsm;
} /* namespace cosm::fsm */

namespace cosm::kin {
class metrics_proxy;
} /* namespace cosm::kin */

namespace cosm::controller {

namespace fs = std::filesystem;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_controller
 * \ingroup controller
 *
 * \brief The implementation of base controller class that the controllers for
 * all robots. It holds all functionality common to both 2D and 3D controllers,
 * as well that some that is stubbed out here, but overridden in derived classes
 * which allows this class to be used as the robot controller handle when
 * rendering QT graphics overlays.
 *
 * It should never be derived from directly; derive from one of the adaptor
 * controllers in the PAL.
 */
class base_controller : public rer::client<base_controller>,
                        public csmetrics::interference_metrics {
 public:
  base_controller(void) RCPPSW_COLD;
  ~base_controller(void) override RCPPSW_COLD;

  base_controller(const base_controller&) = delete;
  base_controller& operator=(const base_controller&) = delete;

  /**
   * \brief Initialize the controller from XML configuration.
   */
  virtual void init(ticpp::Element&) RCPPSW_COLD = 0;

  /**
   * \brief Resut the controller before initialization, after simulation
   * finishes, etc. Should be idempotent, making starting running the controller
   * after calling this function the same as if the controller had just been
   * created.
   */
  virtual void reset(void) RCPPSW_COLD = 0;

  /**
   * \brief Run the main controller loop.
   */
  virtual void control_step(void) = 0;

  /**
   * \brief Get the ID of the entity, which is unique among all entities of the
   * same type in simulation. For real robots, it doesn't have to be unique, but
   * it probably still should be to assist with debugging.
   */
  virtual rtypes::type_uuid entity_id(void) const = 0;

  /**
   * \brief Update the sensing for the robot.
   *
   * - Set the current clock tick.
   *
   * - Update positioning information.
   *
   * \param tick The current clock tick.
   *
   * \param ratio The ratio that should be used to calculate the robot's
   *              discrete position in the arena (should match the ratio used to
   *              create the arena grid).
   */
  virtual void sensing_update(const rtypes::timestep& tick,
                              const rtypes::discretize_ratio& ratio) = 0;

  /**
   * \brief Update the sensing for the robot.
   *
   * - Set the current clock tick.
   *
   * - Update positioning information.
   *
   * \param tick The current simulation clock tick.
   */
  virtual void sensing_update(const rtypes::timestep& tick) = 0;

  /**
   * \brief Convenience function to add timestamp to messages during
   * the control step.
   */
  virtual void mdc_ts_update(void) const = 0;

  /**
   * \brief Convenience function to add footbot ID to salient messages during
   * loop function execution (timestep is already there).
   */
  void ndc_uuid_push(void) const {
    ER_NDC_PUSH("[" + cpal::kRobotNamePrefix +
                rcppsw::to_string(entity_id().v()) + "]");
  }
  void ndc_uuid_pop(void) const { ER_NDC_POP(); }

  /**
   * \brief Return a handle to the \ref rmath::rng used for random
   * number generation by this robot.
   */
  rmath::rng* rng(void) { return m_rng; }

  rmath::vector3d rpos3D(void) const;
  rmath::vector3z dpos3D(void) const;
  rmath::radians azimuth(void) const;
  rmath::radians zenith(void) const;
  rmath::vector2d rpos2D(void) const;
  rmath::vector2z dpos2D(void) const;

  cfsm::supervisor_fsm* supervisor(void) { return m_supervisor.get(); }
  const cfsm::supervisor_fsm* supervisor(void) const {
    return m_supervisor.get();
  }
  const cspatial::interference_tracker* inta_tracker(void) const {
    return m_inta_tracker.get();
  }
  const ckin::metrics_proxy* kin_proxy(void) const { return m_kin_proxy.get(); }

  const class subsystem::base_saa_subsystem* saa(void) const { return m_saa.get(); }

  class subsystem::base_saa_subsystem* saa(void) {
    return m_saa.get();
  }

 protected:
  void saa(std::unique_ptr<subsystem::base_saa_subsystem> saa);


  cspatial::interference_tracker* inta_tracker(void) {
    return m_inta_tracker.get();
  }

  /**
   * \brief Initialize controller output (i.e. where it will log events of
   * interest).
   *
   * Sets up the following log files in the output directory:
   *
   * - cosm.controller -> controller.log
   * - cosm.fsm -> fsm.log
   * - cosm.subsystem.saa -> saa.log
   *
   * \return Absolute path to the output directory.
   */
  virtual fs::path output_init(const cpconfig::output_config* config);

  /**
   * \brief Initialize random number generation for the controller.
   *
   * \param seed The seed to use. -1 results in time seeded RNG, otherwise the
   *             seed value is used.
   * \param category The category of the RNG so that multiple robots can share
   *                 RNG (or not), depending on configuration.
   */
  virtual void rng_init(int seed, const std::string& category);

  void supervisor(std::unique_ptr<cfsm::supervisor_fsm> fsm);

  void inta_tracker(std::unique_ptr<cspatial::interference_tracker> inta);

  void kin_proxy(std::unique_ptr<ckin::metrics_proxy> prox);

 private:
  /* clang-format off */
  rmath::rng*                                     m_rng{nullptr};
  std::unique_ptr<cfsm::supervisor_fsm>           m_supervisor{nullptr};
  std::unique_ptr<cspatial::interference_tracker> m_inta_tracker{nullptr};
  std::unique_ptr<ckin::metrics_proxy>            m_kin_proxy{nullptr};
  std::unique_ptr<subsystem::base_saa_subsystem>  m_saa{nullptr};
  /* clang-format on */

 public:
  /* collision metrics */
  RCPPSW_WRAP_DECLDEF_OVERRIDE(exp_interference, *m_inta_tracker, const);
  RCPPSW_WRAP_DECLDEF_OVERRIDE(entered_interference, *m_inta_tracker, const);
  RCPPSW_WRAP_DECLDEF_OVERRIDE(exited_interference, *m_inta_tracker, const);
  RCPPSW_WRAP_DECLDEF_OVERRIDE(interference_duration, *m_inta_tracker, const);
  RCPPSW_WRAP_DECLDEF_OVERRIDE(interference_loc3D, *m_inta_tracker, const);
};

} /* namespace cosm::controller */
