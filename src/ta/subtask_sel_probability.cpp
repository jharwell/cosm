/**
 * \file subtask_sel_probability.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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
#include "cosm/ta/subtask_sel_probability.hpp"

#include "cosm/ta/config/sigmoid_sel_config.hpp"
#include "cosm/ta/time_estimate.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
subtask_sel_probability::subtask_sel_probability(std::string method)
    : ER_CLIENT_INIT("cosm.ta.subtask_sel_prob"), mc_method(std::move(method)) {
  if (kMethodHarwell2018 == method) {
    sigmoid::init(
        kHARWELL2018_REACTIVITY, kHARWELL2018_OFFSET, kHARWELL2018_GAMMA);
  } else if (kMethodBrutschy2014 == method) {
    sigmoid::init(
        kBRUTSCHY2014_REACTIVITY, kBRUTSCHY2014_OFFSET, kBRUTSCHY2014_GAMMA);
  } else if (kMethodRandom == method) {
  } else {
    ER_FATAL_SENTINEL("Bad method '%s' selected", method.c_str());
  }
}

subtask_sel_probability::subtask_sel_probability(
    const config::sigmoid_sel_config* const config)
    : ER_CLIENT_INIT("cosm.ta.subtask_sel_prob"),
      sigmoid(&config->sigmoid),
      mc_method(config->method) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
double subtask_sel_probability::calc_random(rmath::rng* rng) {
  return eval(rng->uniform(0.0, 1.0));
} /* calc_random() */

double subtask_sel_probability::calc_brutschy2014(const time_estimate& int_est1,
                                                  const time_estimate& int_est2) {
  return eval(calc_sigmoid(int_est1, int_est2));
} /* calc_brutschy2014() */

double subtask_sel_probability::calc_harwell2018(const time_estimate& exec_est1,
                                                 const time_estimate& exec_est2) {
  return eval(calc_sigmoid(exec_est2, exec_est1));
} /* calc_harwell2018() */

double subtask_sel_probability::calc_sigmoid(const time_estimate& est1,
                                             const time_estimate& est2) {
  /*
   * No information available: apply L'Hospital's rule to obtain new terms that
   * ARE defined (the limit resolves to 1).
   */
  double ratio;
  if (0 == est1 && 0 == est2) {
    ratio = 1.0;
  } else if (0 == est1) { /* only est2 defined--math reduces to this */
    ratio = 1.0;
  } else if (0 == est2) { /* only est1 defined--explore! (math limit DNE) */
    ratio = 1.0;
  } else { /* general case: both estimates defined */
    double r_ss = 0;
    if (est1 > est2) {
      r_ss = std::pow(est1.v(), 2) / est2.v();
    } else {
      r_ss = est1.v();
    }
    ratio = est1.v() / r_ss;
  }
  double theta = reactivity() * (ratio - offset());
  return 1.0 / (1 + std::exp(-theta)) * gamma();
} /* calc_sigmoid() */

double subtask_sel_probability::operator()(const time_estimate* subtask1,
                                           const time_estimate* subtask2,
                                           rmath::rng* rng) {
  if (kMethodBrutschy2014 == mc_method) {
    return calc_brutschy2014(*subtask1, *subtask2);
  } else if (kMethodHarwell2018 == mc_method) {
    return calc_harwell2018(*subtask1, *subtask2);
  } else if (kMethodRandom == mc_method) {
    return calc_random(rng);
  }
  ER_FATAL_SENTINEL("Bad method '%s' selected", mc_method.c_str());
  return 0.0;
} /* operator() */

NS_END(ta, cosm);
