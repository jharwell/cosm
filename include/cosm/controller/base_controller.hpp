/**
 * \file base_controller.hpp
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

#ifndef INCLUDE_COSM_CONTROLLER_BASE_CONTROLLER_HPP_
#define INCLUDE_COSM_CONTROLLER_BASE_CONTROLLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <ext/ticpp/ticpp.h>

#include <memory>
#include <string>
#include <typeindex>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/rng.hpp"
#include "rcppsw/types/discretize_ratio.hpp"
#include "rcppsw/types/timestep.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "cosm/cosm.hpp"
#include "cosm/pal/pal.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::fsm {
class supervisor_fsm;
} /* namespace cosm::fsm */

NS_START(cosm, controller);

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
class base_controller : public rer::client<base_controller> {
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
   * \brief Set whether or not a robot is supposed to display it's ID above its
   * head during simulation.
   */
  void display_id(bool b) { m_display_id = b; }

  /**
   * \brief If \c TRUE, then the robot should display its ID above its head
   * during simulation.
   */
  bool display_id(void) const { return m_display_id; }

  /**
   * \brief If \c TRUE, then the robot should display its current LOS during
   * simulation, if it has one.
   */
  bool display_los(void) const { return m_display_los; }

  /**
   * \brief Set whether or not a robot is supposed to display it's LOS during
   * simulation.
   */
  void display_los(bool b) { m_display_los = b; }

  void display_steer2D(bool b) { m_display_steer2D = b; }
  bool display_steer2D(void) const { return m_display_steer2D; }

  /**
   * \brief Update the sensing for the robot.
   *
   * - Set the current clock tick.
   * - Update positioning information.
   *
   * In a real world, each robot would maintain its own clock tick, and overall
   * there would no doubt be considerable skew; this is a simulation hack that
   * makes things much nicer/easier to deal with.
   *
   * \param tick The current simulation clock tick.
   * \param ratio The ratio that should be used to calculate the robot's
   *              discrete position in the arena (should match the ratio used to
   *              create the arena grid)).
   */
  virtual void sensing_update(const rtypes::timestep& tick,
                              const rtypes::discretize_ratio& ratio) = 0;

#if (LIBRA_ER >= LIBRA_ER_ALL)
  /**
   * \brief Convenience function to add footbot ID to salient messages during
   * loop function execution (timestep is already there).
   */
  void ndc_push(void) const {
    ER_NDC_PUSH("[ent" + rcppsw::to_string(entity_id().v()) + "]");
  }
  void ndc_pop(void) const { ER_NDC_POP(); }

#else
  void ndc_push(void) const {}
  void ndc_pop(void) const {}
#endif

  /**
   * \brief Return a handle to the \ref rmath::rng used for random
   * number generation by this robot.
   */
  rmath::rng* rng(void) { return m_rng; }

  cfsm::supervisor_fsm* supervisor(void) { return m_supervisor.get(); }
  const cfsm::supervisor_fsm* supervisor(void) const {
    return m_supervisor.get();
  }

 protected:
  /**
   * \brief Initialize controller output (i.e. where it will log events of
   * interest).
   *
   * \param output_root Absolute or relative path to the output root for the
   *                    robot.
   * \param output_dir Directory name within the output root that things should
   *                   be logged into. This is a separate argument than
   *                   output_root, because there are special values of it that
   *                   have different behavior.
   *
   * Sets up the following log files in the output directory:
   *
   * - cosm.controller -> controller.log
   * - cosm.fsm -> fsm.log
   * - cosm.subsystem.saa -> saa.log
   *
   * \return Absolute path to the output directory.
   */
  std::string output_init(const std::string& output_root,
                          const std::string& output_dir);

  /**
   * \brief Initialize random number generation for the controller.
   *
   * \param seed The seed to use. -1 results in time seeded RNG, otherwise the
   *             seed value is used.
   * \param category The category of the RNG so that multiple robots can share
   *                 RNG (or not), depending on configuration.
   */
  void rng_init(int seed, const std::string& category);

  void supervisor(std::unique_ptr<cfsm::supervisor_fsm> fsm);

 private:
  /* clang-format off */
  bool                                  m_display_id{false};
  bool                                  m_display_steer2D{false};
  bool                                  m_display_los{false};
  rmath::rng*                           m_rng{nullptr};
  std::unique_ptr<cfsm::supervisor_fsm> m_supervisor;
  /* clang-format on */
};

NS_END(controller, cosm);

#endif /* INCLUDE_COSM_CONTROLLER_BASE_CONTROLLER_HPP_ */
