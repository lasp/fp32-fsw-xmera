Executive Summary
-----------------
This attitude guidance module creates a reference attitude message that points in a fixed inertial direction.
All quantities are single precision (float).

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - attRefOutMsg
      - :ref:`AttRefMsgF32Payload`
      - attitude reference output message (only ``sigma_RN`` is set; ``omega_RN_N`` and ``domega_RN_N`` stay zero)


Module Parameters
-----------------

.. list-table:: Module Parameters
    :widths: 20 20 10 50
    :header-rows: 1

    * - Parameter Name
      - Type
      - Default
      - Description
    * - sigma_RN
      - Eigen::Vector3f
      - zero
      - reference-attitude MRP from inertial frame N to reference frame R; must be finite (validated when the
        configuration is built in ``reset()``)


Reference Frame Generation
--------------------------
The modules requires the desired reference orientation in terms of the MRP set :math:`\mathbf{\sigma}_{R N}`.
This input is only set once and does not have to be changed. Let us designate :math:`\mathcal{R}` as the output
generated reference frame. Since the fixed-pointing is inertial:

.. math::
    \mathbf{\sigma}_{RN} = \mathbf{\sigma}_{R N} \\
    \mathbf{\omega}_{RN} = \dot{\mathbf{\omega}}_{RN} = 0

User Guide
----------
The required module configuration is::

    module = inertial3DF32.Inertial3D()
    module.sigma_RN = sigma_input_RN

``sigma_RN`` must be set before ``reset()`` is called: the module builds and validates its immutable configuration
at reset (raising if ``sigma_RN`` is non-finite).
