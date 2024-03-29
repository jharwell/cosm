.. SPDX-License-Identifier:  MIT

.. _ln-cosm-usage-xml-swarm-manager:

===============================
Swarm Manager XML Configuration
===============================

The following root XML tags are defined for swarm management:

.. list-table::
   :widths: 25,50
   :header-rows: 1

   * - Root XML Tag

     - Description

   * - ``output``

     - Parameters for logging simulation metrics/results.

   * - ``convergence``

     - Parameters for computing swarm convergence.

   * - ``arena_map``

     - Parameters for the 2D arena/foraging.

   * - ``temporal_variance``

     - Parameters for temporally varying swarm operating conditions.

   * - ``visualization``

     - Parameters for visualizing robots in various ways during simulation.

   * -  ``oracle_manager``

     - Parameters for providing perfect information to the swarm during
       simulation.

``output``
==========

- Required by: all controllers.
- Required child attributes if present: all.
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: none.

XML configuration:

.. code-block:: XML

    <output
        output_root="output"
        output_dir="__current_date__">
        <metrics>
            ...
        </metrics>
    </output>

- ``output_root`` - The root output directory in which the directories of
  different simulation runs will be placed. The path specified can be relative
  or absolute, and will be created if it does not exist.

- ``output_dir`` - The output directory for the current simulation under
  ``output_root``. If you put the special field ``__current_date__`` here, the
  simulation will get a unique output directory in the form
  ``YYYY-MM-DD:HH-MM``.


``output/metrics``
------------------

- Required by: all controllers.
- Required child attributes if present: [ ``output_dir`` ].
- Required child tags if present: [ ``sinks`` ].
- Optional child attributes: none.
- Optional child tags: none.

XML configuration:

.. code-block:: XML

    <output>
        ...
        <metrics
            output_dir="metrics">
            <sinks>
               ...
            </sinks>
        </metrics>
        ...
    </output>

- ``output_dir`` - Name of directory within the output root that metrics will be
  placed in.

``output/metrics/sinks``
^^^^^^^^^^^^^^^^^^^^^^^^

- Required by: all controllers.
- Required child attributes if present: none.
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: [ ``csv`` ].

XML configuration:

.. code-block:: XML

    <metrics>
        ...
        <sinks>
            <csv>
        </sinks>
        ...
    </metrics>

``output/metrics/sinks/csv``
""""""""""""""""""""""""""""

- Required by: none.
- Required child attributes if present: none.
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: [ ``append``, ``create``, ``truncate`` ].

XML configuration:

.. code-block:: XML

    <csv>
        ...
        <create
             output_interval="INTEGER"
             />
        <append
            output_interval="INTEGER"
            />
        <truncate
            output_interval="INTEGER"
            />

        ...
    </csv>


- ``output_interval`` - Required for all child tags. For ``append``, this
  defines the timestep interval after which metrics will be written out
  (appended) to the specified ``.csv`` created from the provided stem.  For
  ``create``, this defines timestep interval after which metrics will be written
  out to a NEW ``.csv`` file with a unique timestep tag after the provided
  stem. For ``truncate``, this defines the timestep interval after which metrics
  will be written out to a truncated ``.csv`` created from the provided stem;
  that is, each time they are output the results of the previously written out
  metrics are lost.

What collectors can be added under what child tag (id,filename) pairs is defined
in the table below . Not defining them disables metric collection of the given
type for that category.

.. NOTE:: Enabling metric collection of the given type does not `necessarily`
          guarantee that those metrics will be collected for a given
          simulation: the right controller/loop function functionality has to be
          active/enabled as well.

.. list-table::
   :widths: auto
   :header-rows: 1

   * - XML Attribute

     - Description

     - Allowable Output Modes

     - Notes

   * - ``spatial_interference_counts``

     - Metrics capturing # robots entering, currently encountering, and exiting
       the inter-robot interference avoidance state.

     - append

     -

   * - ``spatial_interference_locs2D``

     - Metrics capturing the spatial distribution of inter-robot interference
       locations in 2D in the arena.

     - create,truncate

     -

   * - ``spatial_interference_locs3D``

     - Metrics capturing the spatial distribution of inter-robot interference
       locations in 3D in the arena.

     - create,truncate

     -

   * - ``spatial_movement``

     - Metrics capturing average distance traveled/velocity for different types
       of robot motion.

     - append

     -


   * - ``spatial_nest_zone``

     - Metrics capturing # robots entering, currently in, and exiting, the
       nest. Average time spent in the nest and first time a robot enters the
       nest during simulation (for any reason).

     - append

     -

   * - ``spatial_dist_pos2D``

     - Metrics capturing the spatial distribution of the swarm in 2D space.

     - create,truncate

     -

   * - ``swarm_dist_pos3D``

     - Metrics capturing the spatial distribution of the swarm in 3D space.

     - create,truncate

     -

   * - ``block_acq_counts``

     - Metrics capturing the # robots exploring for, vectoring to, and acquiring
       blocks.

     - append

     -

   * - ``block_acq_locs2D``

     - Metrics capturing the 2D spatial distribution of where robots acquire
       blocks.

     - create,truncate

     -

   * - ``block_acq_explore_locs2D``

     - Metrics capturing the 2D spatial distribution of where robots explore
       for blocks.

     - create,truncate

     -

   * - ``block_acq_explore_locs3D``

     - Metrics capturing the 3D spatial distribution of where robots explore
       for blocks.

     - create,truncate

     -

   * - ``block_acq_vector_locs2D``

     - Metrics capturing the 2D spatial distribution of where robots vector to
       known blocks.

     - create,truncate

     -

   * - ``block_transportee``

     - Metrics capturing the # blocks collected/ # transporters per block for
       different block types.

     - append

     -

   * - ``block_transporter``

     - Metrics capturing robot behavior when transporting blocks to the nest
       (e.g., phototaxis).

     - append

     -

   * - ``task_distribution``

     - Metrics capturing TAB task allocation probabilities/counts.

     - append

     -

   * - ``swarm_convergence``

     - Metrics capturing the results of swarm convergence calculations.

     - append

     -

   * - ``tv_population``

     - Metrics capturing the effect of Poisson processes for governing
       population dynamics (e.g., robot malfunction/repair).

     - append

     -


   * - ``block_distributor``

     - Metrics capturing different aspects of block distribution in general
       (e.g., # clusters).

     - append

     -

   * - ``block_motion``

     - Metrics capturing aspects of free block motion in the arena.

     - append

     -

   * - ``block_clusters``

     - Metrics capturing the size, block count, etc. in block clusters in the
       arena.

     - append

     -

   * - ``nest_acq_strategy``

     - Diagnostic metrics for the strategies robots can use to acquire nests in
       the arena.

     - append

     -

   * - ``battery_state``

     - Diagnostic metrics for robot batteries.

     - append, stream

     -

``convergence``
===============

- Required by: none.
- Required child attributes if present: all.
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: [ ``postional_entropy``, ``task_dist_entropy``,
  ``interactivity``, ``angular_order``, ``velocity`` ].

XML configuration:

.. code-block:: XML

   <convergence>
       <postional_entropy>
       ...
       </positional_entropy>
       <task_dist_entropy>
       ...
       </task_dist_entropy>
       <interactivity>
       ...
       </interactivity>
       <angular_order>
       ...
       </angular_order>
       <velocity>
       ...
       </velocity>
   </convergence>

- ``n_threads`` - How many threads will be used for convergence calculations
  during loop functions.

- ``epsilon`` - Threshold < 1.0 that a convergence measure will be considered
  to have converged when its normalized value is above.

``convergence/positional_entropy``
----------------------------------

A measure of convergence using robot positions, Shannon's entropy definition,
and Balch2000's social entropy measure. If it is defined, only the ``enable``
attribute is required. All other attributes are parsed iff ``enable`` is `true`.

- Required by: none.
- Required child attributes if present: ``enable``.
- Required child tags if present: none.
- Optional child attributes: [ ``horizon``, ``horizon_delta`` ].
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <convergence>
       ...
       <postional_entropy
           enable="false"
           horizon="FLOAT:FLOAT"
           horizon_delta="FLOAT:FLOAT"/>
       ...
   </convergence>


- ``enable`` - If this measure is enabled or not. Very expensive to compute in
  large swarms.

- ``horizon`` - A ``min:max`` pair of distances specifying the min and max
  spatial cluster size that will be used to compute the entropy of robot
  positions. Should be <= arena X,Y dimensions. Only required if ``enable`` is `true`.

- ``horizon_delta`` - Step size for traversing the horizon from min to max. Only
  required if ``enable`` is `true`.


``convergence/interactivity``
-----------------------------

A measure of convergence using nearest neighbor distances.

- Required by: none.
- Required child attributes if present: ``enable``.
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <convergence>
       ...
       <interactivity
           enable="false"/>
       ...
   </convergence>

- ``enable`` - If this measure is enabled or not. Relatively cheap to compute in
  large swarms.

""" ``angular_order``

A measure of convergence using congruence of robot orientations.

- Required by: none.
- Required child attributes if present: ``enable``.
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <convergence>
       ...
       <angular_order
           enable="false"/>
       ...
   </convergence>

- ``enable`` - If this measure is enabled or not. Relatively cheap to compute in
  large swarms.

``convergence/angular_order``
-----------------------------

A measure of convergence using stability of robot task allocations over time.

- Required by: none.
- Required child attributes if present: ``enable``.
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <convergence>
       ...
       <task_dist_entropy
           enable="false"/>
       ...
   </convergence>

- ``enable`` - If this measure is enabled or not. Relatively cheap to compute in
  large swarms.


``convergence/velocity``
------------------------

A measure of convergence using stability of swarm velocity (how much its
geometric center moves) over time.

- Required by: none.
- Required child attributes if present: ``enable``.
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <convergence>
       ...
       <velocity
           enable="false"/>
       ...
   </convergence>

- ``enable`` - If this measure is enabled or not. Relatively cheap to compute in
  large swarms.

``arena_map``
=============

- Required by: all.
- Required child attributes if present: none.
- Required child tags if present: [ ``grid``, ``blocks`` ].
- Optional child attributes: none.
- Optional child tags: none [ ``nests`` ].

XML configuration:

.. code-block:: XML

   <arena_map>
       <grid>
       ...
       </grid>
       <blocks>
       ...
       </blocks>
       <nests>
       ...
       </nests>
   </arena_map>

``arena_map/grid``
------------------

- Required by: all.
- Required child attributes if present: [ ``resolution``, ``size`` ].
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <arena_map>
       ...
       <grid
           resolution="FLOAT"
           size="X, Y"/>
       ...
   </arena_map>

- ``resolution`` - The resolution that the arena will be represented at, in
  terms of the size of grid cells. Must be the same as the value passed to the
  robot controllers.

- ``size`` - The size of the arena.

``arena_map/blocks``
--------------------

- Required by: all.
- Required child attributes if present: none.
- Required child tags if present: [ ``distribution``, ``manifest`` ].
- Optional child attributes: [ ``motion`` ]
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <arena_map>
       ...
       <blocks>
           <distribution>
           ...
           </distribution>
           <motion>
           ...
           </motion>
           <manifest>
           ...
           </manifest>
       </blocks>
       ...
   </arena_map>

``arena_map/blocks/distribution``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- Required by: all.
- Required child attributes if present: ``dist_type``.
- Required child tags if present: none.
- Optional child attributes: [ ``strict_success`` ].
- Optional child tags: [ ``redist_governor``, ``powerlaw`` ].

XML configuration:

.. code-block:: XML

   <blocks>
       ...
       <distribution
       dist_type="random|powerlaw|single_source|dual_source|quad_source"
       strict_success="true">
       ...
       </distribution>
       ...
   </blocks>

- ``dist_type`` - The distribution model for the blocks. When blocks are
  distributed to a new location in the arena and made available for robots to
  pickup (either initially or after a block is deposited in a nest), they are
  placed in the arena in one of the following ways:

  - ``random``: Placed in a random location in the arena.

  - ``powerlaw``: Distributed according to a powerlaw.

  - ``single_source`` - Placed within an arena opposite about 90" of the way
    from the nest to the other side of the arena Assumes horizontal, rectangular
    arena with a single nest.

  - ``dual_source`` - Placed in two sources on either side of a central nest
    Assumes a horizontal, rectangular arena, with a single nest.

  - ``quad_source`` - Placed in 4 sources at each cardinal direction in the
    arena. Assumes a square arena with a single nest.

- ``strict_success`` - Do all blocks need to be successfully distributed when
  distribution is attempted? Useful for scripting when you need to have the same
  " blocks available across a range of arena sizes, and for smaller sizes
  failure to distribute all blocks is OK.

``arena_map/blocks/distribution/redist_governor``
"""""""""""""""""""""""""""""""""""""""""""""""""

- Required by: none.
- Required child attributes if present: ``trigger``.
- Required child tags if present: none.
- Optional child attributes: [ ``recurrence_policy``, ``timestep``, ``block_count`` ].
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <distribution>
       ...
       <redist_governor
           disable_trigger="none"
           recurrence_policy="multi|latch"
           timestep="INTEGER"
           redistribute="true"
           block_count="INTEGER"/>
       ...
   </distribution>


- ``disable_trigger`` - The trigger for (possibly) stopping block
  redistribution:

  - ``none`` - Disables the governor. Whatever the initial state is, it will not
    change.

  - ``timestep`` - Blocks will be redistributed until the specified
                 timestep. This trigger type can be used with the [ ``latch`` ]
                 recurrence policy.

  - ``block_count`` - Blocks will be redistributed until the specified " of
    blocks have been collected. This trigger type can be used with the
    ``latch`` recurrence policy.

  - ``convergence`` - Blocks will be redistributed until the swarm has
    converged. This trigger type can be used with the ``latch``, ``multi``
    recurrence policies.

- ``recurrence_policy`` - The policy for determining how block redistribution
  status can change as the simulation progresses.

  - ``latch`` - Once the specified trigger is tripped, then block
    redistribution will stop permanently.

  - ``multi`` - Blocks will be redistributed as long as the specified trigger
    has not been tripped. Once it has been tripped, block distribution will stop
    until the trigger is no longer tripped, in which case it will resume.

- ``timestep`` - The timestep to stop block redistribution at. Only required if
  ``disable_trigger`` is ``timestep``.

- ``block_count`` - The collection count to stop block redistribution at. Only
  required if ``disable_trigger`` is ``block_count``.

- ``redistribute`` - Should blocks be redistributed initially? If ``false``,
  then all other configuration is optional and ignored.

``arena_map/blocks/distribution/manifest``
""""""""""""""""""""""""""""""""""""""""""

- Required by: all.
- Required child attributes if present: At least one of [ ``n_cube``, ``n_ramp`` ],
  ``unit_dimm``.
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: At most one of [ ``n_cube``, ``n_ramp`` ].

XML configuration:

.. code-block:: XML

    <distribution>
        ...
        <manifest
            n_cube="INTEGER"
            n_ramp="INTEGER"
            unit_dim="FLOAT"/>
        ...
    </distribution>


- ``n_cube`` - " Cube blocks that should be used.

- ``n_ramp`` - " Ramp blocks that should be used.

- ``unit_dim`` - Unit dimension of blocks. Cubes are 1x1 of this, ramps are 2x1 of
  this.

``arena_map/blocks/distribution/powerlaw``
""""""""""""""""""""""""""""""""""""""""""

- Required by: all iff ``dist_type`` is ``powerlaw``.
- Required child attributes if present: [ ``pwr_min``, ``pwr_max``, ``n_clusters`` ].
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <distribution>
       ...
       <powerlaw
           pwr_min="INTEGER"
           pwr_max="INTEGER"
           n_clusters="INTEGER"/>
       ...
   </distribution>

- ``pwr_min`` - Minimum power of 2 for cluster sizes.

- ``pwr_max`` - Maximum power of 2 for cluster sizes.

- ``n_clusters`` - Max " of clusters the arena.

``arena_map/blocks/motion``
"""""""""""""""""""""""""""

- Required by: none.
- Required child attributes if present: ``policy``.
- Required child tags if present: none.
- Optional child attributes: [ ``random_walk_prob`` ].
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <blocks>
       ...
       <motion>
           policy="random_walk"
           prob="FLOAT"
       </motion>
       ...
   </blocks>

- ``policy`` - If the ``<motion>`` tag is present, how should blocks move in the
  arena ?

  - ``random_walk`` - Block motion is a pure random walk which is executed on
    each block each timestep with probability ``random_walk_prob``.

- ``random_walk_prob`` - The probability to perform a random walk for a block on
  a timestep. Only required if ``policy`` is ``random_walk``. Must be >= 0 and
  <= 1.0.

``arena_map/nests``
^^^^^^^^^^^^^^^^^^^

- Required by: none.
- Required child attributes if present: none.
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: [ ``nest`` ].

XML configuration:

.. code-block:: XML

    <arena_map>
        ...
        <nests>
            <nest>
                ...
            </nest>
            <nest>
                ...
            </nest>
            ...
        </nests>
        ...
    </arena_map>

``arena_map/nests/nest``
""""""""""""""""""""""""

- Required by: none.
- Required child attributes if present: [ ``dims``, ``center`` ].
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <nests>
       <nest dims="X, Y"
             center="X, Y"/>
       <nest dims="X, Y"
             center="X, Y"/>
       ...
   </nests>

- ``dims`` - The dimensions of the nest. Must be specified in a tuple like so:
  ``0.5, 0.5``.

- ``center`` - Location for center of the nest (nest is a square).  Must be
  specified in a tuple like so: ``1.5, 1.5``.


``temporal_variance``
=====================

- Required by: none.
- Required child attributes if present: none.
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: [ ``env_dynamics``, ``population_dynamics`` ].

XML configuration:

.. code-block:: XML

   <temporal_variance>
       <env_dynamics>
       ...
       </env_dynamics>
       <population_dynamics>
       ...
       </population_dynamics>
   </temporal_variance>


``temporal_variance/env_dynamics``
----------------------------------

- Required by: none.
- Required child attributes if present: none.
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: [ ``blocks``, ``motion_throttle`` ].

Subsections in this section make use of the ``waveform`` XML configuration block:

.. code-block:: XML

   <waveform
       type="none|sine|square|sawtooth|constant"
       frequency="FLOAT"
       amplitude="FLOAT"
       offset="FLOAT"
       phase="FLOAT"/>


- ``type`` - The type of the waveform. ``none`` disables the waveform.

Other parameters are self explanatory. ``phase`` is specified in radians.

XML configuration:

.. code-block:: XML

   <env_dynamics>
       <motion_throttle>
       ...
       </motion_throttle>
       <blocks>
           <manip_penalty>
           ...
           </manip_penalty>
           <carry_throttle>
           ...
           </carry_throttle>
           </blocks>
   </env_dynamics>

``temporal_variance/env_dynamics/motion_throttle``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- Required by: none.
- Required child attributes if present: none.
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <env_dynamics>
       ...
       <motion_throttle>
           <!-- [waveform config] -->
       </motion_throttle>
       ...
   </env_dynamics>


- ``Waveform`` - Parameters defining the waveform of the robot motion throttle
  which is applied regardless of whether or not they are carrying a block.

``temporal_variance/env_dynamics/blocks/manip_penalty``
"""""""""""""""""""""""""""""""""""""""""""""""""""""""

- Required by: none.
- Required child attributes if present: none.
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <blocks>
       ...
       <manipulation_penalty>
           <!-- [waveform config] -->
       </manipulation_penalty>
       ...
   </blocks>

- ``Waveform`` - Parameters defining the waveform of block manipulation penalty
  (picking up/dropping that does not involve caches).

``temporal_variance/env_dynamics/blocks/carry_throttle``
""""""""""""""""""""""""""""""""""""""""""""""""""""""""

- Required by: none.
- Required child attributes if present: none.
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <blocks>
       ...
       <carry_throttle>
           <!-- [waveform config] -->
       </carry_throttle>
       ...
   </blocks>

- ``Waveform`` - Parameters defining the waveform of block carry penalty (how
  much slower robots move when carrying a block).


``temporal_variance/population_dynamics``
-----------------------------------------

- Required by: none.
- Required child attributes if present: none.
- Required child tags if present: none.
- Optional child attributes: [ ``birth_mu``, ``death_lambda`` ,
  ``repair_lambda``, ``repair_mu`` ].
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <temporal_variance>
       ...
       <population_dynamics
           birth_mu="0.0"
           death_lambda="0.0"
           repair_lambda="0.0"
           repair_mu="0.0"
           max_size="0"/>
       ...
   </temporal_variance>

All parameters have the default values shown above if omitted.

- ``birth_mu`` - Parameter for pure birth Poisson process describing the rate at
  which new robots will be introduced into the simulation, up to ``max_size``
  robots.

- ``death_lambda`` - Parameter for pure death Poisson process describing the
  rate at which existing robots will be permanently removed from simulation.

- ``repair_lambda`` - Parameter for general birth-death Poisson process
  describing the rate at which robots will be temporarily removed from
  simulation in order to simulate being repaired (i.e. added to repair queue).

- ``repair_mu`` - Parameter for general birth-death Poisson process
  describing the rate at which robots which have been temporarily removed from
  the simulation will be restored (i.e. removed from repair queue).

- ``max_size`` - The maximum swarm size achievable using the pure birth process.

``oracle_manager``
==================

- Required by: none.
- Required child attributes if present: none.
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: [ ``tasking_oracle``, ``entities_oracle`` ].

XML configuration:

.. code-block:: XML

   <oracle_manager>
       <tasking_oracle>
       ...
       </tasking_oracle>
       <entities_oracle>
       ...
       </entities_oracle>
   </oracle_manager>


``oracle_manager/tasking_oracle``
---------------------------------

- Required by: none.
- Required child attributes if present: none.
- Required child tags if present: none.
- Optional child attributes: [ ``task_exec_ests``, ``task_interface_ests`` ].
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <oracle_manager>
       ...
       <tasking_oracle
           task_exec_ests="false"
           task_interface_ests="false"/>
       ...
   </oracle_manager>


All attributes default as shown above if omitted.

- ``task_exec_ests`` - If enabled, then this will inject perfect estimates of
  task execution time based on the performance of the entire swarm into each
  robot when it performs task allocation.

- ``task_interface_ests`` - If enabled, then this will inject perfect estimates
  of task interface time based on the performance of the entire swarm into each
  robot when it performs task allocation.

``oracle_manager/entities_oracle``
----------------------------------

- Required by: none.
- Required child attributes if present: none.
- Required child tags if present: none.
- Optional child attributes: [ ``blocks``, ``caches`` ].
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <oracle_manager>
       ...
       <entities_oracle
           blocks="false"
           caches="false"/>
       ...
   </oracle_manager>

- ``blocks`` - Inject perfect knowledge of all block locations into the
  swarm every timestep.

- ``caches`` - Inject perfect knowledge of all cache locations into the
  swarm every timestep.

``visualization``
=================

- Required by: none.
- Required child attributes if present: none.
- Required child tags if present: none.
- Optional child attributes: [ ``robot_id``, ``robot_los``, ``robot_task``, ``block_id`` ].
- Optional child tags: none.

XML configuration:

.. code-block:: XML

    <visulation
        robot_id="false"
        robot_los="false"
        robot_task="false"
        block_id="false"/>


Omitted attributes default to the values shown above.

- ``robot_id`` - If `true`, robot id is displayed above each robot during
  simulation. Default if omitted: `false`.

- ``robot_los`` - If `true`, each robot's approximate line of sight is displayed
  as a red wireframe square during simulation. Only applicable to MDPO
  controllers. Default if omitted: `false`.

- ``robot_task`` - If `true`, the current task each robot is executing is
  displayed above it. Default if omitted: `false`.

- ``block_id`` - If `true`, each block's id displayed above it during
  simulation. Default if omitted: `false`.
