Reproducing and sharing results
===============================

Scenario links
--------------

Every parameter on the simulator page is encoded in the page address after ``#``, with
failed drones numbered from 1. Parameters missing from an address take their default
values, so a short address also works. For example::

   simulator.html#N=6&F=1.2&rho=0.7&phase=turn&t_f=3.8&s_lat=4

opens a 6-drone team carrying a heavy payload (:math:`\rho_L = 0.7`) that loses drones 1 and 2
in a turn with a 320 ms nominal switching delay. **Copy scenario link** produces such an address
for the scenario on screen.

Determinism
-----------

A scenario together with its seed fully determines a run: wind gusts come from a seeded
generator, and the counterfactual branches reuse the same random numbers. The same link
gives the same result in every browser. A batch is determined by its first seed, the
number of scenarios and the boxes ticked. The downloaded CSV records every sampled
parameter, the switching budgets and each system's outcome.

Running the core outside the browser
------------------------------------

The simulation core (``_static/dronelift/sim-core.js``) has no browser dependencies and
runs in Node.js:

.. code-block:: javascript

   const S = require('./docs/_static/dronelift/sim-core.js');
   const p = S.defaultParams();          // 5 drones, drones 1-2 fail in a turn
   p.N = 5; p.F = [0, 2]; p.rho = 0.7;   // change team and payload
   const r = S.compareSystems(p);
   console.log(r.hp.success, r.simplex.success, r.budgets.HP, r.budgets.SC);

``tests/batch.js`` in the repository runs the Monte Carlo batch from the command line.
