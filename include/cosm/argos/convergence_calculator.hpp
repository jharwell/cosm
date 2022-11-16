/**
 * \file convergence_calculator.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "rcppsw/patterns/decorator/decorator.hpp"

#include "cosm/convergence/convergence_calculator.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::pal::argos {
class swarm_manager_adaptor;
class controller2D_adaptor;
class controllerQ3D_adaptor;
} /* namespace cosm::pal::argos */

NS_START(cosm, argos);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class convergence_calculator
 * \ingroup argos
 *
 * \brief Implements the necessary callbacks from \ref
 * cconvergence::convergence_calculator to calculate convergence in the ARGoS
 * environment.
 */
template <class TController>
class convergence_calculator final
    : public rer::client<convergence_calculator<TController>>,
      public rpdecorator::decorator<cconvergence::convergence_calculator> {
 public:
  convergence_calculator(const cconvconfig::convergence_config* config,
                         cpargos::swarm_manager_adaptor* sm) RCPPSW_COLD;
  ~convergence_calculator(void) override RCPPSW_COLD = default;

  /* Not copy constructible/assignable by default */
  convergence_calculator(const convergence_calculator&) = delete;
  convergence_calculator& operator=(const convergence_calculator&) = delete;

  RCPPSW_DECORATE_DECLDEF(update);
  RCPPSW_DECORATE_DECLDEF(converged);
  RCPPSW_DECORATE_DECLDEF(reset_metrics);
  RCPPSW_DECORATE_DECLDEF(task_dist_entropy_init);

 private:
  std::vector<double> calc_robot_nn(size_t n_threads) const;
  std::vector<rmath::radians> calc_robot_headings2D(size_t n_threads) const;
  std::vector<rmath::vector2d> calc_robot_positions(size_t n_threads) const;

  /* clang-format off */
  cpargos::swarm_manager_adaptor* m_sm;
  /* clang-format on */
};

NS_END(argos, cosm);
