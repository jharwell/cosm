/**
 * \file loctree.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/arena/ds/loctree.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, arena, ds);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
template <typename TEntity>
void loctree::do_update(const TEntity* ent) {
  remove(ent);
  decoratee().insert(ent->id(),
                     rds::make_rtree_box(ent->ranchor2D(),
                                         ent->ranchor2D() + ent->rdims2D()));
} /* do_update() */

size_t loctree::remove(const crepr::base_entity* ent) {
  return decoratee().remove(ent->id());
} /* remove() */

/*******************************************************************************
 * Template Instantiations
 ******************************************************************************/
template void loctree::do_update(const crepr::unicell_entity2D*);
template void loctree::do_update(const crepr::unicell_entity3D*);

NS_END(ds, arena, cosm);
