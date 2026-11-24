System model
============

Team and payload
----------------

Drones :math:`i \in \mathcal N = \{1,\dots,N\}` fly at slot azimuths :math:`\psi_i =
\psi_0 + 2\pi i/N` of a regular polygon around a point-mass payload, each on a cable
of length :math:`\ell` at nominal angle :math:`\theta_0` from vertical. The formation
radius is :math:`\ell\sin\theta_0` and the drones fly :math:`\ell\cos\theta_0` above
the payload. The payload obeys

.. math::

   m_L \dot v_L = \sum_{i \in \mathcal M} T_i\, q_i - m_L g\, e_3 + w,

where :math:`\mathcal M` is the set of members, :math:`q_i` is the unit cable
direction from the payload to drone :math:`i`, :math:`T_i \ge 0` is the cable tension
(zero when slack) and :math:`w` is the wind disturbance.

Payload configuration
---------------------

The load is set either by mass :math:`m_L` or by the **load ratio**

.. math::

   \rho_L = \frac{m_L (g + a_{\mathrm{ref}})}{N \cos\theta_0\, (\bar f - m_d g)},

the fraction of the team's spare vertical lift that the payload uses. The paper's
experiments use :math:`\rho_L \in \{0.40, 0.55, 0.70\}`. After :math:`|F|` failures the
same payload loads the :math:`n = N - |F|` survivors at :math:`\rho_L N / n`.

The capacity panel reports the following closed-form quantities.

* Per-member hover thrust in a regular :math:`k`-gon:
  :math:`f^\star_k = \sqrt{(m_d g + T\cos\theta_0)^2 + (T\sin\theta_0)^2}` with
  :math:`T = m_L g / (k\cos\theta_0)`, and the safety factor :math:`\sigma_k = \bar f / f^\star_k`.
* Descent capacity :math:`\mathrm{Cap}_D(S)\colon\ \cos\theta_0 \sum_{i\in S}(\bar f - m_d g) \ge m_L (g - \bar a)`.
  Scenarios that violate it are outside the scope and carry no guarantee.
* Recovery capacity :math:`\mathrm{Cap}_R` uses :math:`g + a_{\mathrm{ref}}` and a 10 % margin.

:math:`\sigma_n \ge 1` certifies only the end state. The simulator shows what happens during the switch.

Fixed parameters
----------------

.. list-table::
   :header-rows: 1
   :widths: 18 52 30

   * - Symbol
     - Meaning
     - Value
   * - :math:`m_d`
     - drone mass (Crazyflie 2.x)
     - 27 g
   * - :math:`\bar f`
     - usable thrust (85 % of motor maximum)
     - 0.506 N
   * - :math:`d_{\mathrm{sep}}`
     - minimum separation (S1)
     - 0.10 m
   * - :math:`h_{\min}`
     - minimum drone height (S2)
     - 0.12 m
   * - :math:`\bar\theta_{\mathrm{tilt}}`
     - maximum tilt (S3)
     - 60°
   * - :math:`\bar a`
     - maximum payload downward acceleration under FS (S6)
     - 0.6 g
   * - :math:`\delta_{\mathrm{HP}},\ \delta_{\mathrm{SC}}`
     - promise tubes
     - 0.30 m, 0.60 m
   * - :math:`D_{\max}`
     - maximum delay under SC
     - 4 s
   * - :math:`\mu_{\mathrm{pos}},\ \mu_{\mathrm{del}}`
     - watchdog margins
     - 0.05 m, 0.3 s
   * - :math:`T_{\mathrm h}`
     - evaluation horizon after the failure
     - 9 s
   * - :math:`a_{\mathrm{ref}}`
     - reference acceleration bound in :math:`\rho_L` (browser assumption)
     - 0.6 m/s²

Scenario parameters
-------------------

These are the controls on the simulator page. The batch draws them from the listed distributions.

.. list-table::
   :header-rows: 1
   :widths: 18 52 30

   * - Symbol
     - Meaning
     - Batch distribution
   * - :math:`N`
     - team size at start
     - :math:`\{3,\dots,7\}`
   * - :math:`|F|`, :math:`F`
     - failure count and pattern (crash)
     - :math:`\{1,\dots,N-2\}`, uniform over dihedral strata
   * - :math:`\psi_0`
     - formation yaw
     - :math:`\mathcal U[0, 360^\circ)`
   * - :math:`\rho_L`
     - load ratio
     - :math:`\{0.40, 0.55, 0.70\}`
   * - :math:`s_\Lambda`, :math:`j`
     - latency scale and jitter
     - :math:`\{0.5,1,2,4,8,16\}`, :math:`\mathrm{LogN}(0, 0.2^2)`
   * - phase
     - flight phase at failure
     - cruise, acceleration, turn
   * - :math:`v_0`, :math:`a_0`
     - cruise speed, acceleration
     - :math:`\mathcal U[0.3, 0.6]` m/s, m/s²
   * - :math:`r_{\mathrm{turn}}`
     - turn radius, direction ±1
     - :math:`\mathcal U[1.5, 3.0]` m
   * - :math:`z_0`
     - payload cruise altitude
     - :math:`\mathcal U[1.3, 1.6]` m
   * - :math:`w`
     - calm, or gusts (mean 0.5 m/s, std. 0.5 m/s, correlation 1 s)
     - :math:`\{0, 1\}`
   * - :math:`t_f`
     - failure time: cruise [2.0, 3.0] s, acceleration [3.1, 3.6] s, turn [3.4, 4.4] s
     - uniform in window
