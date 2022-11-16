/**
 * \file polled_task.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ta/polled_task.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
polled_task::~polled_task(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void polled_task::exec_estimate_init(const rmath::rangez& bounds,
                                     rmath::rng* rng) {
  executable_task::exec_estimate_init(rtypes::timestep(rng->uniform(bounds)));
} /* exec_estimate_init() */

NS_END(ta, cosm);
