/**
 * \file cell2D_unknown.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"

#include "cosm/ds/operations/cell2D_op.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::ds {
class cell2D;
} // namespace cosm::ds

namespace cosm::fsm {
class cell2D_fsm;
}

namespace cosm::ds::operations {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cell2D_unknown
 * \ingroup ds operations detail
 *
 * \brief Created whenever a cell within an occupancy grid needs to go into an
 * unknown state.
 *
 * This happens in two cases:
 *
 * 1. After its relevance expires.
 * 2. Before the robot sees it for the first time (ala Fog of War).
 *
 * This class should never be instantiated, only derived from. To visit \ref
 * cell2D objects, use \ref cell2D_unknown_visitor.
 */
class cell2D_unknown : public cell2D_op, public rer::client<cell2D_unknown> {
 private:
  struct visit_typelist_impl {
    using inherited = cell2D_op::visit_typelist;
    using value = boost::mpl::joint_view<inherited::type>;
  };

 public:
  using visit_typelist = cell2D_op::visit_typelist;

  explicit cell2D_unknown(const rmath::vector2z& coord)
      : cell2D_op(coord), ER_CLIENT_INIT("cosm.ds.operations.cell2D_unknown") {}

  void visit(cds::cell2D& cell);
  void visit(fsm::cell2D_fsm& fsm);
};

/**
 * \brief We use the precise visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using cell2D_unknown_visitor = rpvisitor::filtered_visitor<cell2D_unknown>;

} /* namespace cosm::ds::operations */

