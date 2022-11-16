/**
 * \file goal_acq_metrics.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <utility>

#include "rcppsw/metrics/base_metrics.hpp"
#include "rcppsw/math/vector3.hpp"
#include "rcppsw/types/named_type.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, spatial, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class goal_acq_metrics
 * \ingroup spatial metrics
 *
 * \brief Interface defining what metrics that should be collected from FSMs as
 * they attempt to acquire a goal (site/object of interest) in SOME way (driving
 * to it directly, exploring for it, etc).
 */
class goal_acq_metrics : public virtual rmetrics::base_metrics {
 public:
  /**
   * \brief A pair of booleans, with the first one indicating that the robot is
   * exploring for its goal, and the second one (only valid if the first is \c
   * TRUE) indicating if it is a "true" exploration (i.e. the robot truly does
   * not know of any instances of its target goal type), as opposed to exploring
   * because all of the known instances of its goal type are deemed unsuitable
   * for whatever reason.
   */
  struct exp_status {
    bool is_exploring{false};
    bool is_true{false};
  };

  /**
   * \brief A strong named type representing the goal the robot is currently
   * trying to acquire. A -1 value is used to indicate that no goal is currently
   * being acquired.
   */

  using goal_type = rtypes::named_type<int, struct goal_type_tag>;

  goal_acq_metrics(void) = default;
  ~goal_acq_metrics(void) override = default;

  /**
   * \brief Return the type of acq that is currently being
   * performed.
   *
   * \return The acq type, or -1 if no acquisition is currently in progress.
   */
  virtual goal_type acquisition_goal(void) const = 0;

  /**
   * \brief Output only defined if \ref goal_type is not -1.
   *
   * \return \ref exp_status.
   */
  virtual exp_status is_exploring_for_goal(void) const = 0;

  /**
   * \brief Output only defined if \ref goal_type is not -1. If \c TRUE, then
   * the robot is vectoring towards its goal (i.e., it knows where it is).
   */
  virtual bool is_vectoring_to_goal(void) const = 0;

  /**
   * \brief If \c TRUE, then the robot has arrived at its goal, and is waiting
   * for some sort of signal from the simulation so that it can start executing
   * the next part of its current FSM as part of its current task.
   */
  virtual bool goal_acquired(void) const = 0;

  /**
   * \brief Return the UUID of the entity the robot THINKS it has acquired, once
   * \ref goal_acquired() returns \c TRUE. Only valid while \ref goal_acquired()
   * returns \c TRUE.
   *
   * If the acquisition was not for an entity, but a location within the arena
   * (or something else), then this function should return \ref
   * rtypes::constants::kNoUUID.
   */
  virtual rtypes::type_uuid entity_acquired_id(void) const = 0;

  /**
   * \brief When \ref goal_acquired() returns \c TRUE, then this should return
   * the location of the goal that was acquired.
   */
  virtual rmath::vector3z acquisition_loc3D(void) const = 0;

  /**
   * \brief When \ref is_exploring_for_goal() returns \c TRUE, then this should
   * return the robot's current position as it explores for its goal. If the
   * robot is only moving in 2D, then the Z component should always be 0.
   */
  virtual rmath::vector3z explore_loc3D(void) const = 0;

  /**
   * \brief When \ref is_vectoring_to_goal() returns \c TRUE, then this should
   * return the robot's current discrete position as it vectors to its
   * goal. If the robot is only moving in 2D, then the Z component should always
   * be 0.
   */
  virtual rmath::vector3z vector_loc3D(void) const = 0;
};

/*******************************************************************************
 * Operators
 ******************************************************************************/
template<typename TInt>
bool operator==(const TInt& other, const goal_acq_metrics::goal_type& goal) {
  return goal.v() == other;
}

template<typename TInt>
bool operator!=(const TInt& other, const goal_acq_metrics::goal_type& goal) {
  return goal.v() != other;
}

NS_END(metrics, spatial, cosm);

