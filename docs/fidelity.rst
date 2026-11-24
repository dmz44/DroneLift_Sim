Relation to the paper's simulator
=================================

The paper's results come from a PyBullet simulation (gym-pybullet-drones, Crazyflie CF2X
models). That code cannot run in a browser, so this site re-implements the closed loop on
reduced-order dynamics. Use the site to **explore the mechanism** (how payload configuration, team
size, failure pattern and switching delay interact) rather than to reproduce the paper's tables
to the decimal.

What is the same
----------------

* Team model: regular polygon, point-mass payload, crash failures, cable release,
  irrelevance of failed drones, survivors only in every constraint.
* Fixed parameters: drone mass, usable thrust, cable length and angle, every safety threshold,
  promise tubes, watchdog margins, horizon (:doc:`model`).
* Scenario distribution: every row of the paper's scenario table, including latency scales,
  jitter, flight phases, failure-time windows, gusts and the dihedral failure strata.
* Runtime logic: stale HP until the switch, the latency chain, the 200 ms blend, Reform, the
  switching-budget definition and label sweep, the decision rule with margin
  :math:`\lambda`, watchdogs W1–W2, and the success predicate.
* Comparison protocol: the four systems share the pre-failure trajectory and use common
  random numbers after the failure.

What differs
------------

.. list-table::
   :header-rows: 1
   :widths: 28 36 36

   * - Aspect
     - Paper (PyBullet)
     - Browser
   * - Drone dynamics
     - rigid body, four motors, Crazyflie PID cascade
     - point mass with a bounded thrust vector (tilt ≤ 60°, :math:`\|f\| \le \bar f`)
       and a 40 ms first-order lag standing in for the attitude loop
   * - Cables
     - PyBullet constraint model
     - massless spring–damper (60 N/m, 0.3 N·s/m) that goes slack in compression
   * - Selector (ML-1)
     - learned DeepSets network, calibrated with Learn-then-Test
     - exact budget labels from the in-browser sweep (perfect selector)
   * - SC parameters (ML-2)
     - learned
     - user-set sliders (default: the paper's hand-set swing-study vector)
   * - Integration
     - PyBullet at 240 Hz physics
     - semi-implicit Euler at 1 kHz
   * - Watchdogs
     - W1–W5 specified, W1–W2 implemented
     - W1–W2

Not modeled in either simulator: cable mass, rotor downwash between neighbours, sensing
noise, and contact between a released drone and the team (assumption F4).

Expect the same ordering as the paper, not the same values: high-level-only control violates
most often, and Simplex approaches always-FS safety while keeping more missions in HP or SC.
The batch on :doc:`simulator` lets you check this for any slice of the scenario space.

Reference batch
---------------

For orientation, here is a 200-scenario batch over the full scenario distribution (first seed 1,
default safety-controller parameters, :math:`\lambda = 0`). It is reproducible on
:doc:`simulator` or with ``node tests/batch.js 200``.

.. list-table::
   :header-rows: 1
   :widths: 40 30 30

   * - System
     - Violation rate
     - Mission kept (safe, ends in HP or SC)
   * - High-level only (HP)
     - 37.5 %
     - 62.5 %
   * - Always SC
     - 27.5 %
     - 72.5 %
   * - Always FS
     - 4.0 %
     - 0.0 %
   * - Simplex (oracle budgets)
     - 4.0 %
     - 68.5 %

Simplex matches the failsafe's violation rate while keeping about two thirds of the missions
running. The always-FS baseline keeps none.
