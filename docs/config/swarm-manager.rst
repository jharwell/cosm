Swarm Manager XML Configuration
================================

The following root XML tags are defined for swarm management:

+------------------------+-----------------------------------------------------------------------+
| Root XML tag           | Description                                                           |
+------------------------+-----------------------------------------------------------------------+
| ``output``             | Parameters for logging simulation metrics/results.                    |
+------------------------+-----------------------------------------------------------------------+
| ``convergence``        | Parameters for computing swarm convergence.                           |
+------------------------+-----------------------------------------------------------------------+
| ``visualization``      | Parameters for visualizing robots in various ways during simulation.  |
+------------------------+-----------------------------------------------------------------------+
| ``temporal_variance``  | Parameters for temporally varying swarm operating conditions.         |
+------------------------+-----------------------------------------------------------------------+

``output``
----------

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
^^^^^^^^^^^^^^^^^^

- Required by: all controllers.
- Required child attributes if present: [ ``output_dir``, ``collect_interval`` ].
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <output>
       ...
       <metrics
           output_dir="metrics"
           output_interval="INTEGER">
           ...
           ...
           ...
           ...
       </metrics>
       ...
   </output>

- ``output_dir`` - Name of directory within the output root that metrics will be
  placed in.

- ``output_interval`` - The timestep interval after which statistics will be
  reset. Gathering statistics on a single timestep of a long simulation is
  generally not useful; hence this field.

Any of the attributes can be added under the ``metrics`` tag in place of one of
the ``<...>`` above. Not defining them disables metric collection of the given
type.

+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| XML attribute                                  | Description                                                                   | Additional Notes                                 |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``fsm_collision_counts``                       | Counts of robots entering, are in, and exiting the collision avoidance state. |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``fsm_collision_locs``                         | Spatial distribution of collision avoidance locations in the arena.           |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``fsm_movement``                               | Swarm average distance traveled/velocity.                                     |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``block_acq_counts``                           | Counts of robots exploring for, vectoring to, and acquiring blocks.           |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``block_acq_locs``                             | Spatial distribution of where robots acquire blocks.                          |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``block_acq_explore_locs``                     | Spatial distribution of robots exploring for blocks.                          |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``block_acq_vector_locs``                      | Spatial distribution of robots vectoring to blocks.                           |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``block_transport``                            | # blocks collected/ # transporters.                                           |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``block_manipulation``                         | Free block pickup/drop counts/penalties.                                      |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``task_distribution``                          | TAB task allocation probabilities/counts.                                     |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``perception_dpo``                             | Metrics from each robots' decaying pheromone store.                           |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``perception_mdpo``                            | Metrics from each robot's internal map of the arena.                          |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``swarm_dist_pos2D``                           | Swarm distribution in 2D space.                                               |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``swarm_convergence``                          | Results of swarm convergence calculations.                                    | Requires convergence calculations to be enabled. |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``tv_environment``                             | Waveforms of the penalties applied to the swarm.                              | Output every timestep.                           |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``tv_population``                              | Poisson processes for governing population dynamics.                          |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+

``convergence``
---------------

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
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

### ``angular_order``

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
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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


``temporal_variance``
---------------------

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
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- Required by: none.
- Required child attributes if present: none.
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: [ ``manip_penalty``, ``carry_throttle`` ].

Subsections in this section make use of the ``waveform`` XML configuration block:

.. code-block:: XML

   <waveform
       type="Null|Sine|Square|Sawtooth|Constant"
       frequency="FLOAT"
       amplitude="FLOAT"
       offset="FLOAT"
       phase="FLOAT"/>


- ``type`` - The type of the waveform. ``Null`` disables the waveform.

Other parameters are self explanatory. ``phase`` is specified in radians.

XML configuration:

.. code-block:: XML

   <env_dynamics>
       <blocks>
           <manip_penalty>
           ...
           </manip_penalty>
           <carry_throttle>
           ...
           </carry_throttle>
           </blocks>
   </env_dynamics>

``temporal_variance/env_dynamics/blocks/manip_penalty``
#######################################################

- Required by: none.
- Required child attributes if present: none.
- Required child tags if present: ``waveform``.
- Optional child attributes: none.
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <blocks>
       ...
       <manipulation_penalty>
       <waveform>
           ...
       </waveform>
       </manipulation_penalty>
       ...
   </blocks>

- ``waveform`` - Parameters defining the waveform of block manipulation penalty
  (picking up/dropping that does not involve caches).

``temporal_variance/env_dynamics/blocks/carry_throttle``
########################################################

- Required by: none.
- Required child attributes if present: none.
- Required child tags if present: ``waveform``.
- Optional child attributes: none.
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <blocks>
       ...
       <carry_throttle>
       <waveform>
       ...
       </waveform>
       </carry_throttle>
       ...
   </blocks>

- ``waveform`` - Parameters defining the waveform of block carry penalty (how
  much slower robots move when carrying a block).


``temporal_variance/population_dynamics``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

``visualization``
-----------------

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
