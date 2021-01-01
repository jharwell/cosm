/**
 * \file argos_convergence_calculator.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/pal/argos_convergence_calculator.hpp"

#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

#include "rcppsw/algorithm/closest_pair2D.hpp"
#include "rcppsw/math/vector2.hpp"

#include "cosm/pal/argos_controller2D_adaptor.hpp"
#include "cosm/pal/argos_controllerQ3D_adaptor.hpp"
#include "cosm/pal/argos_swarm_iterator.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, pal);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
template <class TController>
argos_convergence_calculator<TController>::argos_convergence_calculator(
    const cconvconfig::convergence_config* config,
    cpal::argos_sm_adaptor* sm)
    : ER_CLIENT_INIT("cosm.pal.argos_convergence_calculator"),
      decorator(config),
      m_sm(sm) {
  if (config->ang_order.enable) {
    decoratee().angular_order_init(
        std::bind(&argos_convergence_calculator::calc_robot_headings2D,
                  this,
                  std::placeholders::_1));
  }
  if (config->interactivity.enable) {
    decoratee().interactivity_init(
        std::bind(&argos_convergence_calculator::calc_robot_nn,
                  this,
                  std::placeholders::_1));
  }
  if (config->pos_entropy.enable) {
    decoratee().positional_entropy_init(
        std::bind(&argos_convergence_calculator::calc_robot_positions,
                  this,
                  std::placeholders::_1));
  }
  if (config->velocity.enable) {
    decoratee().velocity_init(
        std::bind(&argos_convergence_calculator::calc_robot_positions,
                  this,
                  std::placeholders::_1));
  }
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
template <class TController>
std::vector<double> argos_convergence_calculator<TController>::calc_robot_nn(
    RCPPSW_UNUSED uint n_threads) const {
  std::vector<rmath::vector2d> v;
  auto cb = [&](auto* robot) {
    v.push_back({ robot->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                  robot->GetEmbodiedEntity().GetOriginAnchor().Position.GetY() });
  };
  cpal::argos_swarm_iterator::robots<argos::CFootBotEntity,
                                     cpal::iteration_order::ekSTATIC>(
      m_sm, cb, kARGoSRobotType);

  /*
   * For each closest pair of robots we find, we add the corresponding distance
   * TWICE to our results vector, because 2 robots i and j are each other's
   * closest robots (if they were not, they would not have been returned by the
   * algorithm).
   */
  std::vector<double> res;
  size_t n_robots = m_sm->GetSpace().GetEntitiesByType(kARGoSRobotType).size();

#pragma omp parallel for num_threads(n_threads)
  for (size_t i = 0; i < n_robots / 2; ++i) {
    auto dist_func = std::bind(
        &rmath::vector2d::distance, std::placeholders::_1, std::placeholders::_2);
    auto pts = ralg::closest_pair2D<rmath::vector2d>()("recursive", v, dist_func);
    size_t old = v.size();
#pragma omp critical
    {
      v.erase(std::remove_if(v.begin(),
                             v.end(),
                             [&](const auto& pt) {
                               return pt == pts.p1 || pt == pts.p2;
                             }),
              v.end());

      ER_ASSERT(old == v.size() + 2,
                "Closest pair of points not removed from set");
      res.push_back(pts.dist);
      res.push_back(pts.dist);
    }
  } /* for(i..) */

  return res;
} /* calc_robot_nn() */

template <class TController>
std::vector<rmath::radians>
argos_convergence_calculator<TController>::calc_robot_headings2D(uint) const {
  std::vector<rmath::radians> v;

  auto cb = [&](const auto* controller) { v.push_back(controller->heading2D()); };
  cpal::argos_swarm_iterator::controllers<argos::CFootBotEntity,
                                          TController,
                                          cpal::iteration_order::ekSTATIC>(
      m_sm, cb, kARGoSRobotType);
  return v;
} /* calc_robot_headings2D() */

template <class TController>
std::vector<rmath::vector2d>
argos_convergence_calculator<TController>::calc_robot_positions(uint) const {
  std::vector<rmath::vector2d> v;

  auto cb = [&](const auto* controller) { v.push_back(controller->rpos2D()); };
  cpal::argos_swarm_iterator::controllers<argos::CFootBotEntity,
                                          TController,
                                          cpal::iteration_order::ekSTATIC>(
      m_sm, cb, kARGoSRobotType);
  return v;
} /* calc_robot_positions() */

/*******************************************************************************
 * Template Instantiations
 ******************************************************************************/
template class argos_convergence_calculator<cpal::argos_controller2D_adaptor>;
template class argos_convergence_calculator<cpal::argos_controllerQ3D_adaptor>;

NS_END(pal, cosm);
