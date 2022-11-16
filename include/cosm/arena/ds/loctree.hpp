/**
 * \file loctree.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/ds/rtree.hpp"
#include "rcppsw/types/type_uuid.hpp"
#include "rcppsw/patterns/decorator/decorator.hpp"
#include "rcppsw/math/vector2.hpp"

#include "cosm/repr/unicell_entity2D.hpp"
#include "cosm/repr/unicell_entity3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, arena, ds, detail);

using rtree_spec_type = rds::rtree_spec<rmath::vector2d,
                                        rds::rtree_box<rmath::vector2d>,
                                        rtypes::type_uuid>;
NS_END(detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class loctree
 * \ingroup arena ds
 *
 * \brief Store an Rtree representation of a type of entities in the arena for
 * fast querying.
 *
 * \note You can't mix multiple types of entities, and the \ref
 * rtypes::type_uuid does not guarantee uniqueness across types (duh).
 */
class loctree final : public rpdecorator::decorator<rds::rtree<detail::rtree_spec_type>> {

 private:
  template<typename TEntity>
  void do_update(const TEntity* ent);

 public:
  loctree(void) = default;

  /* Not move/copy constructable/assignable by default */
  loctree(const loctree&) = delete;
  const loctree& operator=(const loctree&) = delete;
  loctree(loctree&&) = delete;
  loctree& operator=(loctree&&) = delete;

  /*
   * \brief Update entity location query tree by removing the old tree entry if
   * it exists and add the new one.
   */
  void update(const crepr::unicell_entity2D* ent) { do_update(ent); }

  /*
   * \brief Update entity location query tree by removing the old tree entry if
   * it exists and add the new one.
   */
  void update(const crepr::unicell_entity3D* ent) { do_update(ent); }

  size_t remove(const crepr::base_entity* ent);

  RCPPSW_DECORATE_DECLDEF(intersections, const);
  RCPPSW_DECORATE_DECLDEF(contains, const);
  RCPPSW_DECORATE_DECLDEF(begin, const);
  RCPPSW_DECORATE_DECLDEF(end, const);
  RCPPSW_DECORATE_DECLDEF(size, const);
};

NS_END(ds, arena, cosm);
