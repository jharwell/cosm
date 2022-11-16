/**
 * \file cell3D.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ds/cell3D.hpp"

#include "cosm/repr/sim_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, ds);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cell3D::cell3D(void) { decoratee().init(); }

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
crepr::sim_block3D* cell3D::block(void) const {
  return dynamic_cast<crepr::sim_block3D*>(m_entity);
} /* block3D() */

crepr::sim_block3D* cell3D::block(void) {
  return dynamic_cast<crepr::sim_block3D*>(m_entity);
} /* block3D() */

NS_END(ds, cosm);
