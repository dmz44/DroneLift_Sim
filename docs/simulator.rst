Interactive simulator
=====================

Set a scenario, then press **Run simulation**. Drag the 3D view to orbit and scroll to
zoom. Hover over a plot to read values, and click a plot to seek the replay to that
instant. Each run updates the page address, so **Copy scenario link** shares the exact
scenario.

.. raw:: html

   <div id="dronelift-app">
     <noscript><p><strong>This page needs JavaScript</strong> to run the simulator.</p></noscript>
   </div>

How to read the result
----------------------

* **Safe run** means the success predicate of :doc:`runtime` held. Every surviving drone
  stayed safe from the failure to the end of the 9 s horizon, and the final mode kept its promise.
  A run that ends with the failsafe landing the payload counts as safe, but the mission
  is abandoned.
* The **switching-budget table** shows, for each mode, whether the team still reaches a
  safe state when that mode takes over after a given delay. The budget
  :math:`\tau_m` is the largest such delay. The column outlined in black brackets this run's
  switching delay :math:`\Delta`.
* The grey strip above each plot shows the active mode (HP, SC, FS). The dashed red
  line marks the failure time.

Monte Carlo batch
-----------------

Draw random scenarios from the paper's distribution (Table 2 of the paper). Each scenario
runs under all four systems with common random numbers. Tick a box to hold that part of
the scenario at the values set above, for example to study one payload configuration
across failure patterns. About 0.5 s per scenario on a laptop.

.. raw:: html

   <div id="dronelift-batch"></div>
