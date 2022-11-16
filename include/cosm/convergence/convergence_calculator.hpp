/**
 * \file convergence_calculator.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/optional.hpp>
#include <memory>
#include <vector>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/radians.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/mpl/typelist.hpp"

#include "cosm/convergence/config/convergence_config.hpp"
#include "cosm/convergence/metrics/convergence_metrics.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace rcppsw::ds {
template <typename Typelist>
class type_map;
} /* namespace rcppsw::ds */

NS_START(cosm, convergence);

class angular_order;
class interactivity;
class positional_entropy;
class task_dist_entropy;
class velocity;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class convergence_calculator
 * \ingroup convergence
 *
 * \brief Convenience class for managing calculation of swarm convergence using
 * any enabled methods.
 *
 * Takes a set of optional callbacks for during construction for calculating the
 * various quantities needed for convergence calculations (if a specific type of
 * convergence calculation is enabled, then you obviously need to pass a valid
 * callback to calculate the necessary input data).
 */
class convergence_calculator final : public metrics::convergence_metrics,
                                     public rer::client<convergence_calculator> {
 public:
  /**
   * \brief Callback function that returns a vector of robot headings (1 per
   * robot). Used to calculate swarm angular order.
   *
   * Takes a single integer argument specifying the # OpenMP threads to be
   * used, per configuration.
   */
  using headings_calc_cb_type =
      std::function<std::vector<rmath::radians>(size_t)>;

  /**
   * \brief Callback function that returns a vector of nearest neighbor
   * distances (1 per robot). Used to calculate swarm interactivity.
   *
   * Takes a single integer argument specifying the # OpenMP threads to be
   * used, per configuration.
   */
  using nn_calc_cb_type = std::function<std::vector<double>(size_t)>;

  /**
   * \brief Callback function that returns a vector of robot positions (1 per
   * robot). Used to calculate swarm positional entropy and velocity (SEPARATE
   * calls per timestep). The calling application should cache the results of
   * the computation passed to the calculator internally if separate calls to
   * obtain the positions of each robot is expensive.
   *
   * Takes a single integer argument specifying the # OpenMP threads to be
   * used, per configuration.
   */
  using pos_calc_cb_type = std::function<std::vector<rmath::vector2d>(size_t)>;

  /**
   * \brief Callback function that returns a vector of robot tasks (as unique
   * non-negative numbers, 1 per robot). Used to calculate swarm task
   * distribution entropy.
   *
   * Tasks a single integer argument specifying the # OpenMP threads to be used,
   * per configuration.
   */
  using tasks_calc_cb_type = std::function<std::vector<int>(size_t)>;

  explicit convergence_calculator(const config::convergence_config* config);
  ~convergence_calculator(void) override;

  /* convergence metrics */
  conv_status_t swarm_interactivity(void) const override;
  conv_status_t swarm_angular_order(void) const override;
  conv_status_t swarm_positional_entropy(void) const override;
  conv_status_t swarm_task_dist_entropy(void) const override;
  conv_status_t swarm_velocity(void) const override;
  double swarm_conv_epsilon(void) const override { return mc_config.epsilon; }
  void reset_metrics(void) override;

  /**
   * \brief Set the callback for calculating \ref angular_order. In order to
   * actually calculate it time \ref update is called, it must have also been
   * enabled in configuration.
   */
  void angular_order_init(const headings_calc_cb_type& cb);

  /**
   * \brief Set the callback for calculating \ref interactivity. In order to
   * actually calculate it time \ref update is called, it must have also been
   * enabled in configuration.
   */
  void interactivity_init(const nn_calc_cb_type& cb);

  /**
   * \brief Set the callback for calculating \ref task_dist_entropy. In order to
   * actually calculate it time \ref update is called, it must have also been
   * enabled in configuration.
   */
  void task_dist_entropy_init(const tasks_calc_cb_type& cb);

  /**
   * \brief Set the callback for calculating \ref positional_entropy. In order
   * to actually calculate it time \ref update is called, it must have also been
   * enabled in configuration.
   */
  void positional_entropy_init(const pos_calc_cb_type& cb);

  /**
   * \brief Set the callback for calculating \ref velocity. In order to
   * actually calculate it time \ref update is called, it must have also been
   * enabled in configuration.
   */
  void velocity_init(const pos_calc_cb_type& cb);

  /**
   * \brief Return swarm convergence status in an OR fashion (i.e. if ANY of the
   * configured methods say convergence has occured, return \c TRUE).
   */
  bool converged(void) const RCPPSW_PURE;

  /**
   * \brief Update convergence calculations for the current timestep
   */
  void update(void);

 private:
  using measure_typelist = rmpl::typelist<positional_entropy,
                                          task_dist_entropy,
                                          angular_order,
                                          interactivity,
                                          velocity>;
  /* clang-format off */
  const config::convergence_config                 mc_config;

  std::unique_ptr<rds::type_map<measure_typelist>> m_measures{nullptr};
  boost::optional<headings_calc_cb_type>           m_headings_calc{nullptr};
  boost::optional<nn_calc_cb_type>                 m_nn_calc{nullptr};
  boost::optional<pos_calc_cb_type>                m_pos_calc{nullptr};
  boost::optional<tasks_calc_cb_type>              m_tasks_calc{nullptr};
  /* clang-format on */
};

NS_END(convergence, cosm);
