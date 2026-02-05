#
#   FSW Setup Utilities for Thrusters
#

from xmera.architecture import messaging


thr_list = []


def create(
        r_thrust_B,
        t_hat_thrust_B,
        F_max
    ):
    """
    This function is called to setup a FSW RW device in python, and adds it to the of RW
    devices in rwList[].  This list is accessible from the parent python script that
    imported this rw library script, and thus any particular value can be over-ridden
    by the user.

    Args:
        r_thrust_B: position of thruster in spacecraft body frame
        t_hat_thrust_B: direction of thrust vector in B frame
        F_max: maximum thrust force value

    """
    global thr_list

    # create the blank Thruster object
    thrPointer = messaging.THRConfigMsgF32Payload()

    thrPointer.rThrust_B = r_thrust_B
    thrPointer.tHatThrust_B = t_hat_thrust_B
    thrPointer.maxThrust = F_max

    # add RW to the list of RW devices
    thr_list.append(thrPointer)

    return


def writeConfigMessage():
    """
    This function should be called after all devices are created with create()
    It creates the C-class container for the array of RW devices, and attaches
    this container to the spacecraft object
    :return:
    """
    global thr_list

    thrClass = messaging.THRArrayConfigMsgF32Payload()
    thrClass.thrusters = thr_list
    thrClass.numThrusters = len(thr_list)
    thrConfigInMsg = messaging.THRArrayConfigMsgF32().write(thrClass)
    thrConfigInMsg.this.disown()

    return thrConfigInMsg

def clearSetup():
    global thr_list

    thr_list = []

    return

def getNumOfDevices():
    return len(thr_list)
