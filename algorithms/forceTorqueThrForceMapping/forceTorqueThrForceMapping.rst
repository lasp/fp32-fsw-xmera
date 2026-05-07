.. raw:: latex

    {\LARGE \textbf{forceTorqueThrForceMapping}}

Executive Summary
-----------------
This module maps commanded forces and torques defined in the body frame of the spacecraft to a set of thrusters. It is
capable of handling Center of Mass (CoM) offsets and non-controllable axes. In contrast to :ref:`thrForceMapping`, this
module only handles on-pulsing, but not off-pulsing. Further, it provides a single force/torque projection onto the
thrusters and thus lacks some of the robustness features of :ref:`thrForceMapping`.

The commanded force and torque input messages are optional, and the associated vectors are zeroed if no input message
is connected. This provides a general capability to map control torques, forces, or torques and forces onto a set of
thrusters.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages. The module msg connection is set by the user from
python. The msg type contains a link to the message structure definition, while the description provides information
on what this message is used for. Both the ``cmdTorqueInMsg`` and ``cmdForceInMsg`` are optional.

.. list-table:: Module I/O Messages
    :widths: 30 30 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - cmdTorqueInMsg
      - :ref:`CmdTorqueBodyMsgPayload`
      - (optional) vehicle control torque (:math:`\mathbf{L}_{r}`) input message
    * - cmdForceInMsg
      - :ref:`CmdForceBodyMsgPayload`
      - (optional) vehicle control force input message
    * - thrConfigInMsg
      - :ref:`THRArrayConfigMsgPayload`
      - thruster cluster configuration input message
    * - vehConfigInMsg
      - :ref:`VehicleConfigMsgPayload`
      - vehicle configuration input message
    * - thrForceCmdOutMsg
      - :ref:`THRArrayCmdForceMsgPayload`
      - thruster force command output message

Module Assumptions and Limitations
----------------------------------
This module assumes that each thruster only produces positive thrust (on-pulsing only). The pseudo-inverse of the
stacked force/torque mapping matrix :math:`[D]` is computed via a truncated SVD: singular values below the relative
cutoff :math:`\sigma_{\max} \cdot \varepsilon_{f32} \cdot \max(6, N_{\max})` (the fp32 noise floor) are treated as
zero, which projects uncontrollable directions in the 6-D command space out of the result. The resulting solution is
the minimum-norm least-squares projection onto the thruster set, shifted so the minimum thrust is zero. No off-pulsing,
saturation handling, or thruster availability masking is performed within this module.

Module Parameters
-----------------
.. list-table:: Module Parameters
    :widths: 25 25 50
    :header-rows: 1

    * - Parameter
      - Type
      - Description
    * - desiredControlAxes_B
      - ``std::array<bool, 6>``
      - Per-axis controllability assertion in body frame :math:`B`. Entries ``[0..2]`` correspond to
        torque components :math:`\tau_x, \tau_y, \tau_z` and entries ``[3..5]`` correspond to force
        components :math:`F_x, F_y, F_z`. A ``true`` entry asserts that the corresponding axis must
        be controllable by the configured thruster array; the assertion is cross-checked against the
        SVD of :math:`[D]` inside ``computeThrusterMapping()``. If any flagged axis lies outside the
        column space of :math:`[D]`, the module throws ``fs::invalid_argument`` (which surfaces in
        Python as ``RuntimeError``). Default is all-true (full controllability asserted); set entries
        to ``False`` to opt out per axis when the thruster array is intentionally rank-deficient.

Initialization
--------------
The module is configured by::

    module = forceTorqueThrForceMapping.ForceTorqueThrForceMapping()
    module.modelTag = "forceTorqueThrForceMappingTag"
    # Optional controllability assertion (torque xyz, force xyz in body frame B)
    module.desiredControlAxes_B = [True, True, True, True, True, False]

The ``cmdForceInMsg`` and ``cmdTorqueInMsg`` are optional; if not connected the corresponding commanded vector is
treated as zero. The ``thrConfigInMsg`` and ``vehConfigInMsg`` are required::

    fswSetupThrusters.clearSetup()
    for i in range(numThrusters):
        fswSetupThrusters.create(rcsLocationData[i], rcsDirectionData[i], maxThrust)
    thrConfigInMsg = fswSetupThrusters.writeConfigMessage()

    vehConfigInMsgData = messaging.VehicleConfigMsgPayload()
    vehConfigInMsgData.CoM_B = CoM_B
    vehConfigInMsg = messaging.VehicleConfigMsg().write(vehConfigInMsgData)

The relevant messages must then be subscribed to by the module::

    module.cmdTorqueInMsg.subscribeTo(cmdTorqueInMsg)
    module.cmdForceInMsg.subscribeTo(cmdForceInMsg)
    module.thrConfigInMsg.subscribeTo(thrConfigInMsg)
    module.vehConfigInMsg.subscribeTo(vehConfigInMsg)

Detailed Module Description
---------------------------
The following text describes the mathematics behind the ``forceTorqueThrForceMapping`` module.

Force and Torque Mapping
^^^^^^^^^^^^^^^^^^^^^^^^
The desired force and torque are given as :math:`\mathbf{F}_{req}` and :math:`\boldsymbol{\tau}_{req}`, respectively.
These are both stacked into a single vector

.. math::
    :label: eq:augmented_ft

    \begin{bmatrix}
        \boldsymbol{\tau}_{req} \\
        \mathbf{F}_{req}
    \end{bmatrix}

The :math:`i`-th thruster position expressed in spacecraft body-fixed coordinates is given by :math:`\mathbf{r}_{i}`.
The unit direction vector of the thruster force is :math:`\hat{\mathbf{g}}_{t_{i}}`. The thruster force is given as

.. math::
    :label: eq:force_direction

    \mathbf{F}_{i} = F_{i} \hat{\mathbf{g}}_{t_{i}}

The torque produced by each thruster about the body-fixed CoM is

.. math::
    :label: eq:torques

    \boldsymbol{\tau}_{i} = \left((\mathbf{r}_{i} - \mathbf{r}_{\text{COM}}) \times \hat{\mathbf{g}}_{t_{i}}\right) F_{i}
    = \mathbf{d}_{i} F_{i}

The total force and torque on the spacecraft may be represented as

.. math::
    :label: eq:sys_eqs

    \begin{bmatrix}
        \boldsymbol{\tau}_{req} \\
        \mathbf{F}_{req}
    \end{bmatrix}
    =
    \begin{bmatrix}
        \mathbf{d}_{1} \cdots \mathbf{d}_{N} \\
        \hat{\mathbf{g}}_{t_{1}} \cdots \hat{\mathbf{g}}_{t_{N}}
    \end{bmatrix}
    \begin{bmatrix}
        F_{1} \\
        \vdots \\
        F_{N}
    \end{bmatrix}
    = [D] \, \mathbf{F}

The force required by each thruster is computed via the Moore-Penrose pseudo-inverse of :math:`[D]`, formed from
its truncated singular value decomposition :math:`[D] = U \Sigma V^{T}`:

.. math::
    :label: eq:soln

    \mathbf{F} = V \, \tilde{\Sigma}^{+} U^{T}
    \begin{bmatrix}
        \boldsymbol{\tau}_{req} \\
        \mathbf{F}_{req}
    \end{bmatrix}

where :math:`\tilde{\Sigma}^{+}` is the diagonal of inverse singular values with entries below
:math:`\sigma_{\max} \cdot \varepsilon_{f32} \cdot \max(6, N_{\max})` zeroed. This automatically drops any
uncontrollable direction in the 6-D command space — equivalent to removing rows of :math:`[D]` whose contribution lies
entirely in the noise floor — without requiring an explicit row-by-row threshold.

To ensure no commanded thrust is less than zero, the minimum thrust is subtracted from the thrust vector

.. math::
    :label: eq:F_min

    \mathbf{F} = \mathbf{F} - \min(\mathbf{F})

These thrust commands are then written to the output message.

Additional Information
----------------------
The minimum-thrust subtraction in :eq:`eq:F_min` guarantees non-negative thruster commands but does not preserve the
exact commanded force and torque when the raw pseudo-inverse solution contains negative components. In those cases the
system sacrifices some of the requested force/torque fidelity in exchange for producing an on-pulse-only solution.
For rank-deficient :math:`[D]` the minimum-norm solution is returned; uncontrollable directions in the 6-D command
space are silently dropped by the SVD truncation. The optional ``desiredControlAxes_B`` parameter converts that silent
drop into a fail-fast assertion at configuration time: if any axis flagged ``True`` in ``desiredControlAxes_B`` lies
outside the column space of :math:`[D]`, ``computeThrusterMapping()`` throws rather than returning a solution that
ignores the request.
