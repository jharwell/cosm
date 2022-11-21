/**
 * \file convergence_calculator.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/convergence/convergence_calculator.hpp"

#include <boost/variant.hpp>

#include "rcppsw/ds/type_map.hpp"

#include "cosm/convergence/angular_order.hpp"
#include "cosm/convergence/interactivity.hpp"
#include "cosm/convergence/positional_entropy.hpp"
#include "cosm/convergence/task_dist_entropy.hpp"
#include "cosm/convergence/velocity.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::convergence {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct convergence_measure_updater
 * \ingroup convergence
 *
 * \brief Visitor class for mapping a given convergence measure to the actions
 * necessary to update it. Needed because not all convergence measures take the
 * same number/type of parameters. This could also be solved with a parameter
 * base class/derived classes and dynamic casting, but I think this is cleaner.
 *
 * It is passed the callbacks from the calculator, rather than the results, so
 * that it is only if a specific type of convergence calculation is enabled that
 * the necessary input data is generated, as this may be as expensive as the
 * actual convergence calculation itself.
 */
class convergence_measure_updater : public boost::static_visitor<void> {
 public:
  convergence_measure_updater(
      size_t n,
      const boost::optional<convergence_calculator::headings_calc_cb_type>&
          headings_calc,
      const boost::optional<convergence_calculator::nn_calc_cb_type>& nn_calc,
      const boost::optional<convergence_calculator::pos_calc_cb_type>& pos_calc,
      const boost::optional<convergence_calculator::tasks_calc_cb_type>&
          tasks_calc)
      : m_n_threads(n),
        m_headings_calc(headings_calc),
        m_nn_calc(nn_calc),
        m_pos_calc(pos_calc),
        m_tasks_calc(tasks_calc) {}
  void operator()(interactivity& i) {
    if (m_nn_calc) {
      i((*m_nn_calc)(m_n_threads));
    }
  }

  void operator()(angular_order& ang) {
    if (m_headings_calc) {
      ang((*m_headings_calc)(m_n_threads), m_n_threads);
    }
  }

  void operator()(positional_entropy& pos) {
    if (m_pos_calc) {
      pos((*m_pos_calc)(m_n_threads));
    }
  }

  void operator()(velocity& vel) {
    if (m_pos_calc) {
      vel((*m_pos_calc)(m_n_threads));
    }
  }

  void operator()(task_dist_entropy& tdist) {
    if (m_tasks_calc) {
      tdist((*m_tasks_calc)(m_n_threads));
    }
  }

 private:
  /* clang-format off */
  size_t                                                         m_n_threads;
  boost::optional<convergence_calculator::headings_calc_cb_type> m_headings_calc;
  boost::optional<convergence_calculator::nn_calc_cb_type>       m_nn_calc;
  boost::optional<convergence_calculator::pos_calc_cb_type>      m_pos_calc;
  boost::optional<convergence_calculator::tasks_calc_cb_type>    m_tasks_calc;
  /* clang-format on */
};

/**
 * \struct convergence_status_updater
 * \ingroup convergence
 *
 * \brief Visitor class for gather the convergence status of each enabled type
 * of convergence calculation.
 */
struct convergence_status_collator : public boost::static_visitor<bool> {
  template <typename T>
  bool operator()(const T& measure) const {
    return measure.converged();
  }
};

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
convergence_calculator::convergence_calculator(
    const config::convergence_config* config)
    : ER_CLIENT_INIT("rcppsw.swarm.convergence.calculator"), mc_config(*config) {}
convergence_calculator::~convergence_calculator(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void convergence_calculator::angular_order_init(const headings_calc_cb_type& cb) {
  m_headings_calc = boost::make_optional(cb);
  m_measures->emplace(typeid(angular_order), angular_order(mc_config.epsilon));
} /* angular_order_init() */

void convergence_calculator::interactivity_init(const nn_calc_cb_type& cb) {
  m_nn_calc = boost::make_optional(cb);
  m_measures->emplace(typeid(interactivity), interactivity(mc_config.epsilon));
} /* interactivity_init() */

void convergence_calculator::task_dist_entropy_init(const tasks_calc_cb_type& cb) {
  m_tasks_calc = boost::make_optional(cb);
  m_measures->emplace(typeid(task_dist_entropy),
                      task_dist_entropy(mc_config.epsilon));
} /* task_dist_init() */

void convergence_calculator::positional_entropy_init(const pos_calc_cb_type& cb) {
  /* velocity and positional entropy use the same callback */
  if (!m_pos_calc) {
    m_pos_calc = boost::make_optional(cb);
  }
  m_measures->emplace(
      typeid(positional_entropy),
      positional_entropy(
          mc_config.epsilon,
          std::make_unique<raclustering::entropy_eh_omp<rmath::vector2d>>(
              mc_config.n_threads),
          &mc_config.pos_entropy));
} /* positional_entropy_init() */

void convergence_calculator::velocity_init(const pos_calc_cb_type& cb) {
  /* velocity and positional entropy use the same callback */
  if (!m_pos_calc) {
    m_pos_calc = boost::make_optional(cb);
  }
  m_measures->emplace(typeid(velocity), velocity(mc_config.epsilon));
} /* velocity_init() */

void convergence_calculator::update(void) {
  convergence_measure_updater u{
    mc_config.n_threads, m_headings_calc, m_nn_calc, m_pos_calc, m_tasks_calc
  };
  for (auto& m : *m_measures) {
    boost::apply_visitor(u, m.second);
  } /* for(&m..) */
} /* update() */

bool convergence_calculator::converged(void) const {
  bool ret = false;
  for (const auto& m : *m_measures) {
    ret |= boost::apply_visitor(convergence_status_collator(), m.second);
  } /* for(&m..) */
  return ret;
} /* converged() */

convergence_calculator::conv_status_t
convergence_calculator::swarm_interactivity(void) const {
  if (!mc_config.interactivity.enable) {
    return std::make_tuple(0.0, 0.0, false);
  }
  auto& tmp = boost::get<interactivity>(m_measures->at(typeid(interactivity)));
  return std::make_tuple(tmp.raw(), tmp.v(), tmp.converged());
} /* swarm_interactivity() */

convergence_calculator::conv_status_t
convergence_calculator::swarm_angular_order(void) const {
  if (!mc_config.ang_order.enable) {
    return std::make_tuple(0.0, 0.0, false);
  }
  auto& tmp = boost::get<angular_order>(m_measures->at(typeid(angular_order)));
  return std::make_tuple(tmp.raw(), tmp.v(), tmp.converged());
} /* swarm_angular_order() */

convergence_calculator::conv_status_t
convergence_calculator::swarm_positional_entropy(void) const {
  if (!mc_config.pos_entropy.enable) {
    return std::make_tuple(0.0, 0.0, false);
  }
  auto& tmp =
      boost::get<positional_entropy>(m_measures->at(typeid(positional_entropy)));
  return std::make_tuple(tmp.raw(), tmp.v(), tmp.converged());
} /* swarm_positional_entropy() */

convergence_calculator::conv_status_t
convergence_calculator::swarm_task_dist_entropy(void) const {
  if (!mc_config.task_dist_entropy.enable) {
    return std::make_tuple(0.0, 0.0, false);
  }
  auto& tmp =
      boost::get<task_dist_entropy>(m_measures->at(typeid(task_dist_entropy)));
  return std::make_tuple(tmp.raw(), tmp.v(), tmp.converged());
} /* swarm_task_dist_entropy() */

convergence_calculator::conv_status_t
convergence_calculator::swarm_velocity(void) const {
  if (!mc_config.velocity.enable) {
    return std::make_tuple(0.0, 0.0, false);
  }
  auto& tmp = boost::get<velocity>(m_measures->at(typeid(velocity)));
  return std::make_tuple(tmp.raw(), tmp.v(), tmp.converged());
} /* swarm_positional_entropy() */

void convergence_calculator::reset_metrics(void) {
  if (mc_config.interactivity.enable) {
    boost::get<interactivity>(m_measures->at(typeid(interactivity))).reset();
  }
  if (mc_config.ang_order.enable) {
    boost::get<angular_order>(m_measures->at(typeid(angular_order))).reset();
  }
  if (mc_config.pos_entropy.enable) {
    boost::get<positional_entropy>(m_measures->at(typeid(positional_entropy)))
        .reset();
  }
  if (mc_config.task_dist_entropy.enable) {
    boost::get<task_dist_entropy>(m_measures->at(typeid(task_dist_entropy)))
        .reset();
  }
  if (mc_config.velocity.enable) {
    boost::get<velocity>(m_measures->at(typeid(velocity))).reset();
  }
} /* reset_metrics() */

} /* namespace cosm::convergence */
