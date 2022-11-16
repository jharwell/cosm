/**
 * \file interaction_probability.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "rcppsw/math/radians.hpp"
#include "rcppsw/spatial/euclidean_dist.hpp"
#include "rcppsw/math/probability.hpp"
#include "rcppsw/math/vector2.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::flocking {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class interaction_probability
 * \ingroup flocking
 *
 * \brief Calculates the probability of two agents interacting during flocking.
 *
 * From Bagarti2018.
 */
class interaction_probability : public rmath::probability {
 public:
  interaction_probability(const rmath::radians& theta_max,
                          const rspatial::euclidean_dist& mean_interaction_dist);


  /**
   * \brief Calculate the probability
   *
   * \param[in] self_pos Agent position.
   *
   * \param[in] self_vel Agent velocity.
   *
   * \param[in] other_idx Index of the OTHER agent to calculate the
   *                      probability of interaction with in \p fov_agents.
   *
   * \param[in] fov_agents The positions of ALL other agents in the current
   *                       agent's Field of View (FOV).
   */
  rmath::probability operator()(const rmath::vector2d& self_pos,
                                const rmath::vector2d& self_vel,
                                size_t other_idx,
                                const std::vector<rmath::vector2d>& fov_agents);

  /* Not move/copy constructable/assignable by default */
  interaction_probability(const interaction_probability&) = delete;
  interaction_probability& operator=(const interaction_probability&) = delete;
  interaction_probability(interaction_probability&&) = delete;
  interaction_probability& operator=(interaction_probability&&) = delete;

 private:
  rmath::probability calc_for_agent(const rmath::vector2d& pos,
                                    const rmath::vector2d& vel,
                                    const rmath::vector2d fov_agent_pos);
  /* clang-format off */
  const rmath::radians mc_theta_max;
  const rspatial::euclidean_dist mc_mean_interaction_dist;
  /* clang-format on */
};

} /* namespace cosm::flocking */
