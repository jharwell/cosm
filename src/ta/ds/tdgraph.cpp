/**
 * \file tdgraph.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * This file is part of RCPPSW.
 *
 * RCPPSW is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * RCPPSW is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * RCPPSW.  If not, see <http://www.gnu.org/licenses/
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ta/ds/tdgraph.hpp"

#include "cosm/ta/polled_task.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta, ds);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
tdgraph::tdgraph(void) : ER_CLIENT_INIT("cosm.ta.tdgraph") {}

/*******************************************************************************
 * Static Member Functions
 ******************************************************************************/
polled_task* tdgraph::vertex_parent(const tdgraph& graph,
                                    const polled_task* const v) {
  return graph.vertex_parent(v);
} /* vertex_parent() */

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
const polled_task* tdgraph::root(void) const { return m_root; }
polled_task* tdgraph::root(void) { return m_root; }

const polled_task* tdgraph::find_vertex(const std::string& task_name) const {
  return m_impl[*find_vertex_impl(task_name)].get();
} /* find_vertex() */

polled_task* tdgraph::find_vertex(const std::string& task_name) {
  return m_impl[*find_vertex_impl(task_name)].get();
} /* find_vertex() */

const polled_task* tdgraph::find_vertex(int id) const {
  return m_impl[id].get();
} /* find_vertex() */

polled_task* tdgraph::find_vertex(int id) {
  return m_impl[id].get();
} /* find_vertex() */

int tdgraph::vertex_id(const polled_task* const v) const {
  if (nullptr == v) {
    return -1;
  }
  auto it = find_vertex_impl(v);
  if (it == boost::vertices(m_impl).second) {
    ER_WARN("No such vertex %s found in graph", v->name().c_str());
    return -1;
  }
  return *it;
} /* vertex_id() */

int tdgraph::vertex_depth(const polled_task* const v) const {
  if (nullptr == v) {
    return -1;
  }
  auto it = find_vertex_impl(v);
  if (it == boost::vertices(m_impl).second) {
    ER_WARN("No such vertex %s found in graph", v->name().c_str());
    return -1;
  }
  return vertex_depth_impl(v, 0);
} /* vertex_depth() */

void tdgraph::walk(const walk_cb& f) {
  vertex_iterator v_i, v_end;
  boost::tie(v_i, v_end) = boost::vertices(m_impl);
  while (v_i != v_end) {
    f(m_impl[*v_i].get());
    ++v_i;
  } /* while() */
} /* walk() */

void tdgraph::walk(const const_walk_cb& f) const {
  vertex_iterator v_i, v_end;
  boost::tie(v_i, v_end) = boost::vertices(m_impl);
  while (v_i != v_end) {
    f(m_impl[*v_i].get());
    ++v_i;
  } /* while() */
} /* walk() */

size_t tdgraph::vertex_depth_impl(const polled_task* const v, int depth) const {
  /*
   * Only the root's parent is equal to itself.
   */
  if (vertex_parent(v) == v) {
    return depth;
  } else {
    return vertex_depth_impl(vertex_parent(v), depth + 1);
  }
} /* vertex_depth_impl() */

tdgraph::vertex_iterator
tdgraph::find_vertex_impl(const polled_task* const v) const {
  vertex_iterator v_i, v_end;
  boost::tie(v_i, v_end) = boost::vertices(m_impl);
  auto it = std::find_if(v_i, v_end, [&](const tdgraph::vertex_desc& tmp) {
    return v == m_impl[tmp].get();
  });
  return it;
} /* find_vertex_impl() */

tdgraph::vertex_iterator tdgraph::find_vertex_impl(const std::string& v) const {
  vertex_iterator v_i, v_end;
  boost::tie(v_i, v_end) = boost::vertices(m_impl);
  auto it = std::find_if(v_i, v_end, [&](const tdgraph::vertex_desc& tmp) {
    return v == m_impl[tmp]->name();
  });
  return it;
} /* find_vertex_impl() */

polled_task* tdgraph::vertex_parent(const polled_task* const v) const {
  auto found = find_vertex_impl(v);
  if (found == boost::vertices(m_impl).second) {
    ER_WARN("No such vertex %s found in graph", v->name().c_str());
    return nullptr;
  }
  /*
   * Now, we can just look in the incident edges for the vertex and return the
   * one we find (if there is more than 1, that's an error).
   */
  in_edge_iterator ie, ie_end;
  boost::tie(ie, ie_end) = boost::in_edges(*found, m_impl);
  ER_ASSERT(
      1 == ie_end - ie, "Vertex %s has more than 1 parent", v->name().c_str());
  return m_impl[boost::source(*ie, m_impl)].get();
} /* vertex_parent() */

status_t tdgraph::set_root(vertex_type v) {
  vertex_desc new_v;
  ER_CHECK(0 == boost::num_edges(m_impl), "Root already set for graph!");
  m_root = v.get();
  new_v = boost::add_vertex(std::shared_ptr<polled_task>(std::move(v)), m_impl);
  boost::add_edge(new_v, new_v, m_impl); /* parent of root is root */
  return OK;

error:
  return ERROR;
} /* set_root() */

std::vector<polled_task*>
tdgraph::children(const polled_task* const parent) const {
  auto it = find_vertex_impl(parent);
  ER_ASSERT(it != boost::vertices(m_impl).second,
            "No such vertex %s found in graph",
            parent->name().c_str());
  std::vector<polled_task*> kids;
  out_edge_iterator oe, oe_end;

  boost::tie(oe, oe_end) = boost::out_edges(*it, m_impl);
  while (oe != oe_end) {
    kids.push_back(m_impl[boost::target(*oe, m_impl)].get());
    ++oe;
  } /* while() */

  return kids;
} /* children() */

status_t tdgraph::set_children(const std::string& parent,
                               vertex_vector children) {
  return set_children(m_impl[*find_vertex_impl(parent)].get(),
                      std::move(children));
} /* set_children() */

status_t tdgraph::set_children(const polled_task* parent,
                               vertex_vector children) {
  out_edge_iterator oe_start, oe_end;
  auto vertex_d = find_vertex_impl(parent);
  ER_CHECK(vertex_d != boost::vertices(m_impl).second,
           "No such vertex %s in graph",
           parent->name().c_str());

  /* The root always has "children", in the sense it points to itself */
  if (m_impl[*vertex_d].get() != m_root) {
    ER_CHECK(0 == boost::out_degree(*vertex_d, m_impl),
             "Graph vertex %s already has children",
             m_impl[*vertex_d]->name().c_str());
  }

  for (auto& c : children) {
    vertex_desc new_v =
        boost::add_vertex(std::shared_ptr<polled_task>(std::move(c)), m_impl);
    ER_TRACE("Add edge %s -> %s",
             m_impl[*vertex_d]->name().c_str(),
             m_impl[new_v]->name().c_str());
    boost::add_edge(*vertex_d, new_v, m_impl);
  } /* for(c..) */
  return OK;

error:
  return ERROR;
} /* set_children() */

NS_END(ds, ta, cosm);
