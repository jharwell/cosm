/**
 * \file cell2D.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ds/cell2D.hpp"

#include "cosm/arena/repr/base_cache.hpp"
#include "cosm/repr/sim_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, ds);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cell2D::cell2D(void) { decoratee().init(); }

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
crepr::sim_block3D* cell2D::block3D(void) const {
  return dynamic_cast<crepr::sim_block3D*>(m_entity);
} /* block3D() */

crepr::sim_block3D* cell2D::block3D(void) {
  return dynamic_cast<crepr::sim_block3D*>(m_entity);
} /* block3D() */

carepr::base_cache* cell2D::cache(void) const {
  return dynamic_cast<carepr::base_cache*>(m_entity);
} /* cache() */

carepr::base_cache* cell2D::cache(void) {
  return dynamic_cast<carepr::base_cache*>(m_entity);
} /* cache() */

NS_END(ds, cosm);
