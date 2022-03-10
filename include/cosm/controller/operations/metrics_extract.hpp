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
#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/variant/static_visitor.hpp>
#include <utility>

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
 * \tparam TController The type of the controller to extract metrics from.
 * \tparam TAggregator The type of the metrics aggregator to use to extract
 *                         the metrics, derived from \ref
 *                         base_metrics_aggregator.
 *
 * \p TController does not have to be a class template parameter per-se, and
 * could be pushed down to operator(). However, if this is done then the
 * compiler cannot infer the type of the derived controller when trying to apply
 * a specific instantiation of this visitor to a boost::variant. So, we encode
 * the necessary type information in the class template parameters.
 */
template <class TController, class TAggregator>
class RCPPSW_EXPORT metrics_extract : public boost::static_visitor<void> {
 public:
  explicit metrics_extract(TAggregator* const agg) : m_agg(agg) {}

  template<typename ...Args>
  void operator()(const TController* const c, Args&& ...args) const {
    m_agg->collect_from_controller(c, std::forward<Args>(args)...);
  }

 private:
  /* clang-format off */
  TAggregator* const m_agg;
  /* clang-format on */
};

NS_END(operations, controller, cosm);

