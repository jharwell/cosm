/**
 * \file temporal_penalty_handler.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <memory>
#include <mutex>
#include <string>

#include "rcppsw/control/periodic_waveform.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/multithread/lockable.hpp"

#include "cosm/tv/config/temporal_penalty_config.hpp"
#include "cosm/tv/temporal_penalty.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::tv {

/*******************************************************************************
 * Classes
 ******************************************************************************/

/**
 * \class temporal_penalty_handler
 * \ingroup tv
 *
 * \brief The penalty handler for penalties for robots (e.g. how long they have
 * to wait when they pickup/drop a block).
 *
 * Does not do much more than provide the penalty list, and functions for
 * manipulating it to derived classes.
 */
class temporal_penalty_handler : public rer::client<temporal_penalty_handler>,
                                 public rmultithread::lockable {
 public:
  using const_iterator_type =
      typename std::list<temporal_penalty>::const_iterator;

  /**
   * \brief Initialize the penalty handler.
   *
   * \param config Parameters for temporal penalties.
   * \param name The name of the handler, for differentiating handler instances
   *             in logging statements.
   */
  temporal_penalty_handler(
      const ctv::config::temporal_penalty_config* const config,
      const std::string& name);

  ~temporal_penalty_handler(void) override = default;

  /* Not copy assignable/copy constructible by default */
  temporal_penalty_handler& operator=(const temporal_penalty_handler&) = delete;
  temporal_penalty_handler(const temporal_penalty_handler&) = delete;

  /**
   * \brief Get the name of the penalty handler (for debugging)
   */
  const std::string& name(void) const { return mc_name; }

  /**
   * \brief Get the next penalty which will be satisfied from the list.
   */
  temporal_penalty penalty_next(void) const;

  /**
   * \brief Remove the specified penalty from the list once the robot it
   * corresponds to has served its penalty.
   *
   * \param victim The penalty to remove.
   * \param lock Is locking required around penalty list modifications or not?
   *             Should *ALWAYS* be \c TRUE if the function is called external
   *             to this class.
   */
  void penalty_remove(const temporal_penalty& victim, bool lock = true);

  /**
   * \brief Abort a robot's serving of its penalty.
   *
   * This should only be done if the robot aborts its task WHILE also serving a
   * penalty.
   *
   * \param controller The robot to abort the penalty for.
   */
  void penalty_abort(const controller::base_controller& controller);

  /**
   * \brief If \c TRUE, then the specified robot is currently serving a cache
   * penalty.
   *
   * \param controller The controller to check penalty serving for.
   * \param lock Is locking required around penalty list modifications or not?
   *             Should *ALWAYS* be \c TRUE if the function is called external
   *             to this class.
   */
  bool is_serving_penalty(const controller::base_controller& controller,
                          bool lock = true) const RCPPSW_PURE;

  /**
   * \brief Determine if a robot has satisfied the \ref temporal_penalty
   * it is currently serving yet.
   *
   * \param controller The robot to check. If the robot is not currently serving
   *                   a penalty, \c FALSE will be returned.
   *
   * \param timestep The current timestep.
   *
   * \return \c TRUE If the robot is currently waiting AND it has satisfied its
   * penalty.
   */
  bool is_penalty_satisfied(const controller::base_controller& controller,
                            const rtypes::timestep& t) const RCPPSW_PURE;

  const_iterator_type penalty_find(const controller::base_controller& controller,
                                   bool lock = true) const;

  /**
   * \brief Calculate the penalty for a robot to serve for the operation, given
   * the current timestep and the configured penalty waveform.
   */
  rtypes::timestep penalty_calc(const rtypes::timestep& t) const;

 protected:
  rtypes::timestep penalty_add(const controller::base_controller* controller,
                               const rtypes::type_uuid& id,
                               const rtypes::timestep& orig_duration,
                               const rtypes::timestep& start);

 private:
  /*
   * \brief Deconflict penalties such that at most 1 robot finishes
   * serving their penalty per timestep.
   *
   * No locking is needed because this is a private function.
   *
   * \param duration The calculated penalty sans deconfliction. Passed by value
   *                 and modified, in order to make calculations simpler.
   */
  rtypes::timestep penalty_finish_uniqueify(const rtypes::timestep& start,
                                            rtypes::timestep duration) const;

  /* clang-format off */
  const bool                          mc_unique_finish;
  const std::string                   mc_name;

  std::list<temporal_penalty>         m_penalty_list{};
  mutable std::shared_mutex           m_list_mtx{};
  std::unique_ptr<rct::base_waveform> m_waveform;
  /* clang-format on */
};

} /* namespace cosm::tv */
