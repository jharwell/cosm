/**
 * \file swarm_manager.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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
#include "cosm/pal/swarm_manager.hpp"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <sys/resource.h>
#include <chrono>
#include <csignal>

#include <boost/stacktrace.hpp>

#include "rcppsw/math/rngm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, pal);

void ___sighandler(int signum);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
swarm_manager_impl::swarm_manager_impl(void)
: ER_CLIENT_INIT("cosm.pal.swarm_manager") {
  /*
   * For some reason GNU parallel on MSI resets the core dump memory limit to 0
   * when running, even if you run "ulimit -s unlimited" right before. And on
   * development machines, it is easy to forget to run the ulimit cmd before
   * doing a test run to help catch a heisenbug in the act, necessitating
   * re-running and much gnashing of teeth.
   *
   * So, we fix the limit during initialization, so that IF we do crash, we
   * always get a core dump to help diagnose things, regardless of shell
   * configuration.
   */
  signal(SIGSEGV, ___sighandler);
  struct rlimit r;
  getrlimit(RLIMIT_CORE, &r);
  r.rlim_cur = RLIM_INFINITY;
  setrlimit(RLIMIT_CORE, &r);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void swarm_manager_impl::output_init(const std::string& output_root,
                                     const std::string& output_dir) {
  if ("__current_date__" == output_dir) {
    boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
    m_output_root = output_root + "/" + rcppsw::to_string(now.date().year()) +
                    "-" + rcppsw::to_string(now.date().month()) + "-" +
                    rcppsw::to_string(now.date().day()) + ":" +
                    rcppsw::to_string(now.time_of_day().hours()) + "-" +
                    rcppsw::to_string(now.time_of_day().minutes());
  } else {
    m_output_root = output_root + "/" + output_dir;
  }
} /* output_init() */

void swarm_manager_impl::rng_init(const rmath::config::rng_config* config) {
  rmath::rngm::instance().register_type<rmath::rng>("swarm_manager");
  if (nullptr == config || (nullptr != config && -1 == config->seed)) {
    ER_INFO("Using time seeded RNG");
    m_rng = rmath::rngm::instance().create(
        "swarm_manager",
        std::chrono::system_clock::now().time_since_epoch().count());
  } else {
    ER_INFO("Using user seeded RNG");
    m_rng = rmath::rngm::instance().create("swarm_manager", config->seed);
  }
} /* rng_init() */

void ___sighandler(int signum) {
  /* trigger a core dump if we get a segfault or other signal */
  std::signal(signum, SIG_DFL);
  int pid = getpid();
  /*
   * In addition to a core dump, write out a stacktrace for later retrieval as
   * well. Between the core dump and stacktrace, there is a lot of redundancy,
   * but if one doesn't get recorded for some reason, we will still have the
   * other, and more information is better for debugging.
   */
  std::string fname = "./backtrace" + rcppsw::to_string(pid);
  boost::stacktrace::safe_dump_to(fname.c_str());

  /* rethrow signal and terminate */
  ::kill(getpid(), signum);
} /* ___sighandler() */

NS_END(pal, cosm);
