/**
 * \file interaction_probability.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "rcppsw/math/radians.hpp"
#include "rcppsw/spatial/euclidean_dist.hpp"
#include "rcppsw/math/expression.hpp"
#include "rcppsw/math/vector2.hpp"

#include "cosm/cosm.hpp"
#include "cosm/apf2D/boid_vector.hpp"

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
class interaction_probability : public rmath::expression<double> {
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
   * \param[in] others The ALL other agents in the current agents FOV.
   */
  double operator()(const rmath::vector2d& self_pos,
                    const rmath::vector2d& self_vel,
                    size_t other_idx,
                    const capf2D::boid_vectorro& others);

  /* Not move/copy constructable/assignable by default */
  interaction_probability(const interaction_probability&) = delete;
  interaction_probability& operator=(const interaction_probability&) = delete;
  interaction_probability(interaction_probability&&) = delete;
  interaction_probability& operator=(interaction_probability&&) = delete;

 private:
  double calc_for_agent(const rmath::vector2d& pos,
                        const rmath::vector2d& vel,
                        const capf2D::boid* other);
  /* clang-format off */
  const rmath::radians mc_theta_max;
  const rspatial::euclidean_dist mc_mean_interaction_dist;
  /* clang-format on */
};

} /* namespace cosm::flocking */
