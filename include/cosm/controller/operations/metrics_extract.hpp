/**
 * \file metrics_extract.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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
#ifndef INCLUDE_COSM_CONTROLLER_OPERATIONS_METRICS_EXTRACT_HPP_
#define INCLUDE_COSM_CONTROLLER_OPERATIONS_METRICS_EXTRACT_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/variant/static_visitor.hpp>

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, controller, operations);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct metrics_extract
 * \ingroup controller operations
 *
 * \brief Functor to perform metric extraction from a controller on each
 * timestep.
 *
 * \tparam TControllerType The type of the controller to extract metrics from.
 * \tparam TAggregatorType The type of the metrics aggregator to use to extract
 *                         the metrics, derived from \ref
 *                         base_metrics_aggregator.
 *
 * \p TControllerType does not have to be a class template parameter per-se, and
 * could be pushed down to operator(). However, if this is done then the
 * compiler cannot infer the type of the derived controller when trying to apply
 * a specific instantiation of this visitor to a boost::variant. So, we encode
 * the necessary type information in the class template parameters.
 */
template <class TControllerType, class TAggregatorType>
class metrics_extract : public boost::static_visitor<void> {
 public:
  explicit metrics_extract(TAggregatorType* const agg) : m_agg(agg) {}

  void operator()(const TControllerType* const c) const {
    m_agg->collect_from_controller(c);
  }

 private:
  /* clang-format off */
  TAggregatorType* const m_agg;
  /* clang-format on */
};

NS_END(operations, controller, cosm);

#endif /* INCLUDE_COSM_CONTROLLER_OPERATIONS_METRICS_EXTRACT_HPP_ */
