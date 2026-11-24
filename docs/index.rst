DroneLift reviewer simulator
============================

This site accompanies the submission *Fault-Tolerant Multi-Drone Cooperative Heavy Lift*.
It lets reviewers change the **payload configuration** and the **number of drones**,
inject crash failures, and watch how the Simplex runtime switches between its three
controllers. Everything runs in your browser: there is no server, no installation and
no data collection.

.. rubric:: What you can do

* Change the team size :math:`N \in \{3,\dots,7\}` and click drones to choose which ones fail.
* Change the payload: load ratio :math:`\rho_L` or mass :math:`m_L`, cable length :math:`\ell`
  and nominal cable angle :math:`\theta_0`. A static capacity check updates as you type.
* Set the flight phase at failure, the switching latency, wind gusts and the
  safety-controller parameters.
* Simulate one scenario and replay it in 3D, with tracking error, thrust use, cable
  tension and altitude plotted against time.
* Run the same scenario under the four systems compared in the paper
  (high-level only, always SC, always FS, Simplex).
* Run a Monte Carlo batch over the paper's scenario distribution and download the
  per-scenario results as CSV.
* Copy a link that reproduces the exact scenario on screen.

.. rubric:: Start here

Open :doc:`simulator`. A default scenario runs automatically when the page loads: five drones
lose two neighbours in a turn, with a 320 ms switching delay. The high-performance budget
(about 0.24 s) is shorter than the delay, so the runtime selects the safety controller. Press
**Compare all four systems** to see high-level-only control break its 0.30 m tube on the same scenario.

.. important::

   The browser simulator is a **reduced-order re-implementation** of the closed loop in
   the paper, not the PyBullet code used for the paper's tables. It reproduces the
   paper's logic (failure semantics, switching delay, switching budgets, decision rule,
   watchdogs, success predicate) on simplified dynamics, and its numbers differ from the
   paper's. :doc:`fidelity` lists exactly what matches and what differs.

.. toctree::
   :maxdepth: 2
   :caption: Contents

   simulator
   model
   runtime
   fidelity
   reproducing
