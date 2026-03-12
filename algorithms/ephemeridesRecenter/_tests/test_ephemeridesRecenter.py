# MIT License
#
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

import inspect
import os
import numpy as np
import spiceypy

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

from xmera.utilities import SimulationBaseClass
from xmera.fp32 import ephemeridesRecenterF32
from xmera.architecture import messaging

SUN_ID = spiceypy.bodn2c("SUN")
EARTH_ID = spiceypy.bodn2c("EARTH")
MOON_ID = spiceypy.bodn2c("MOON")
MARS_ID = spiceypy.bodn2c("MARS")
SATURN_ID = spiceypy.bodn2c("SATURN")

def test_body_recenter():
    """ Test ephemeridesRecenter. """
    mars_central_body()

def test_moon_recenter():
    """ Test ephemeridesRecenter. """
    moon_central_body()

def test_clear_recenter():
    """ Test ephemeridesRecenter. """
    clearing_values()

def mars_central_body():
    """ Test the ephemeridesRecenter module. Setup a simulation, """

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName,  int(0.5e9)))

    ephemRecenter = ephemeridesRecenterF32.EphemeridesRecenter()
    ephemRecenter.modelTag = "ephemeridesRecenter"
    unitTestSim.AddModelToTask(unitTaskName, ephemRecenter)

    list_of_bodies = []
    # Create bodies to add to the module
    sunBody = ephemeridesRecenterF32.BodyEphemeris()
    sunInputPayload = messaging.EphemerisMsgF32Payload()
    position = [0,0,0]
    velocity = [0,0,0]
    sunInputPayload.r_BdyZero_N = position
    sunInputPayload.v_BdyZero_N = velocity
    sunInputPayload.timeTag = 1234.0
    sunInputMessage = messaging.EphemerisMsgF32().write(sunInputPayload)
    sunBody.inputEphemerisMsg.subscribeTo(sunInputMessage)
    sunBody.bodySpiceId = SUN_ID
    sunBody.originalCentralBodyId = SUN_ID
    list_of_bodies.append(sunBody)

    # Set this message
    earthBody = ephemeridesRecenterF32.BodyEphemeris()
    earthInputPayload = messaging.EphemerisMsgF32Payload()
    position = [1000, -200, 100]
    velocity = [10, 0, -8]
    earthInputPayload.r_BdyZero_N = position
    earthInputPayload.v_BdyZero_N = velocity
    earthInputPayload.timeTag = 1234.0
    earthInputMessage = messaging.EphemerisMsgF32().write(earthInputPayload)
    earthBody.inputEphemerisMsg.subscribeTo(earthInputMessage)
    earthBody.bodySpiceId = EARTH_ID
    earthBody.originalCentralBodyId = SUN_ID
    list_of_bodies.append(earthBody)

    marsBody = ephemeridesRecenterF32.BodyEphemeris()
    marsInputPayload = messaging.EphemerisMsgF32Payload()
    position = [-4000, 3000, 10000]
    velocity = [-1, -2, 1]
    marsInputPayload.r_BdyZero_N = position
    marsInputPayload.v_BdyZero_N = velocity
    marsInputPayload.timeTag = 1234.0
    marsInputMessage = messaging.EphemerisMsgF32().write(marsInputPayload)
    marsBody.inputEphemerisMsg.subscribeTo(marsInputMessage)
    marsBody.bodySpiceId = MARS_ID
    marsBody.originalCentralBodyId = SUN_ID
    list_of_bodies.append(marsBody)

    moonBody = ephemeridesRecenterF32.BodyEphemeris()
    moonInputPayload = messaging.EphemerisMsgF32Payload()
    position = [-50, 30, 100]
    velocity = [-0.5, -0.2, 0.1]
    moonInputPayload.r_BdyZero_N = position
    moonInputPayload.v_BdyZero_N = velocity
    moonInputPayload.timeTag = 1234.0
    moonInputMessage = messaging.EphemerisMsgF32().write(moonInputPayload)
    moonBody.inputEphemerisMsg.subscribeTo(moonInputMessage)
    moonBody.bodySpiceId = MOON_ID
    moonBody.originalCentralBodyId = EARTH_ID
    list_of_bodies.append(moonBody)

    dataLogList = list()
    names = []
    for body in list_of_bodies:
        ephemRecenter.addBodyEphemerisToRecenter(body)
        names.append(body.bodySpiceId)
    for body in list_of_bodies:
        index = ephemRecenter.getBodyIndexFromId(body.bodySpiceId)
        rec = ephemRecenter.recenteredEphemerisOutputMsgs[index].recorder()
        dataLogList.append(rec)
        unitTestSim.AddModelToTask(unitTaskName, rec)

    ephemRecenter.setPreviousCommonZeroBase(SUN_ID)
    ephemRecenter.setNewZeroBase(MARS_ID)
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(0)
    unitTestSim.ExecuteSimulation()

    mars_r_recentered = np.array(dataLogList[2].r_BdyZero_N).reshape([3,])
    mars_v_recentered = np.array(dataLogList[2].v_BdyZero_N).reshape([3,])
    mars_time_tag = dataLogList[2].timeTag

    np.testing.assert_array_almost_equal(mars_r_recentered, np.zeros(3), 15, "Mars position error")
    np.testing.assert_array_almost_equal(mars_v_recentered, np.zeros(3), 15, "Mars velocity error")
    np.testing.assert_equal(mars_time_tag, marsInputPayload.timeTag, "Mars time tag error")

    earth_r_recentered = np.array(dataLogList[1].r_BdyZero_N).reshape([3,])
    earth_v_recentered = np.array(dataLogList[1].v_BdyZero_N).reshape([3,])
    earth_time_tag = dataLogList[1].timeTag

    np.testing.assert_array_almost_equal(earth_r_recentered,
                                  np.array(earthInputPayload.r_BdyZero_N) - np.array(marsInputPayload.r_BdyZero_N),
                                         15,
                                  "Earth position error")
    np.testing.assert_array_almost_equal(earth_v_recentered,
                                  np.array(earthInputPayload.v_BdyZero_N) - np.array(marsInputPayload.v_BdyZero_N),
                                         15,
                                  "Earth velocity error")
    np.testing.assert_equal(earth_time_tag, earthInputPayload.timeTag, "Earth time tag error")

    sun_r_recentered = np.array(dataLogList[0].r_BdyZero_N).reshape([3,])
    sun_v_recentered = np.array(dataLogList[0].v_BdyZero_N).reshape([3,])
    sun_time_tag = dataLogList[0].timeTag

    np.testing.assert_array_almost_equal(sun_r_recentered,
                                  np.array(sunInputPayload.r_BdyZero_N) - np.array(marsInputPayload.r_BdyZero_N),
                                         15,
                                  "Sun position error")
    np.testing.assert_array_almost_equal(sun_v_recentered,
                                  np.array(sunInputPayload.v_BdyZero_N) - np.array(marsInputPayload.v_BdyZero_N),
                                         15,
                                  "Sun velocity error")
    np.testing.assert_equal(sun_time_tag, sunInputPayload.timeTag, "Sun time tag error")

    moon_r_recentered = np.array(dataLogList[3].r_BdyZero_N).reshape([3,])
    moon_v_recentered = np.array(dataLogList[3].v_BdyZero_N).reshape([3,])
    moon_time_tag = dataLogList[3].timeTag

    np.testing.assert_array_almost_equal(moon_r_recentered,
                                  np.array(earthInputPayload.r_BdyZero_N) - np.array(marsInputPayload.r_BdyZero_N)
                                  + np.array(moonInputPayload.r_BdyZero_N),
                                         15,
                                  "Moon position error")
    np.testing.assert_array_almost_equal(moon_v_recentered,
                                  np.array(earthInputPayload.v_BdyZero_N) - np.array(marsInputPayload.v_BdyZero_N)
                                  +np.array(moonInputPayload.v_BdyZero_N),
                                         15,
                                  "Moon velocity error")
    np.testing.assert_equal(moon_time_tag, moonInputPayload.timeTag, "Moon time tag error")

def moon_central_body():
    """ Test the ephemeridesRecenter module. Setup a simulation, """

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName,  int(0.5e9)))

    ephemRecenter = ephemeridesRecenterF32.EphemeridesRecenter()
    ephemRecenter.modelTag = "ephemeridesRecenter"
    unitTestSim.AddModelToTask(unitTaskName, ephemRecenter)

    list_of_bodies = []
    # Create bodies to add to the module
    sunBody = ephemeridesRecenterF32.BodyEphemeris()
    sunInputPayload = messaging.EphemerisMsgF32Payload()
    position = [80000, 10000, -9000]
    velocity = [-5, 2, -0.9]
    sunInputPayload.r_BdyZero_N = position
    sunInputPayload.v_BdyZero_N = velocity
    sunInputPayload.timeTag = 1234.0
    sunInputMessage = messaging.EphemerisMsgF32().write(sunInputPayload)
    sunBody.inputEphemerisMsg.subscribeTo(sunInputMessage)
    sunBody.bodySpiceId = SUN_ID
    sunBody.originalCentralBodyId = SATURN_ID
    list_of_bodies.append(sunBody)

    # Set this message
    earthBody = ephemeridesRecenterF32.BodyEphemeris()
    earthInputPayload = messaging.EphemerisMsgF32Payload()
    position = [1000, -200, 100]
    velocity = [10, 0, -8]
    earthInputPayload.r_BdyZero_N = position
    earthInputPayload.v_BdyZero_N = velocity
    earthInputPayload.timeTag = 1234.0
    earthInputMessage = messaging.EphemerisMsgF32().write(earthInputPayload)
    earthBody.inputEphemerisMsg.subscribeTo(earthInputMessage)
    earthBody.bodySpiceId = EARTH_ID
    earthBody.originalCentralBodyId = SATURN_ID
    list_of_bodies.append(earthBody)

    marsBody = ephemeridesRecenterF32.BodyEphemeris()
    marsInputPayload = messaging.EphemerisMsgF32Payload()
    position = [-4000, 3000, 10000]
    velocity = [-1, -2, 1]
    marsInputPayload.r_BdyZero_N = position
    marsInputPayload.v_BdyZero_N = velocity
    marsInputPayload.timeTag = 1234.0
    marsInputMessage = messaging.EphemerisMsgF32().write(marsInputPayload)
    marsBody.inputEphemerisMsg.subscribeTo(marsInputMessage)
    marsBody.bodySpiceId = MARS_ID
    marsBody.originalCentralBodyId = SATURN_ID
    list_of_bodies.append(marsBody)

    moonBody = ephemeridesRecenterF32.BodyEphemeris()
    moonInputPayload = messaging.EphemerisMsgF32Payload()
    position = [-50, 30, 100]
    velocity = [-0.5, -0.2, 0.1]
    moonInputPayload.r_BdyZero_N = position
    moonInputPayload.v_BdyZero_N = velocity
    moonInputPayload.timeTag = 1234.0
    moonInputMessage = messaging.EphemerisMsgF32().write(moonInputPayload)
    moonBody.inputEphemerisMsg.subscribeTo(moonInputMessage)
    moonBody.bodySpiceId = MOON_ID
    moonBody.originalCentralBodyId = EARTH_ID
    list_of_bodies.append(moonBody)

    saturnBody = ephemeridesRecenterF32.BodyEphemeris()
    saturnInputPayload = messaging.EphemerisMsgF32Payload()
    position = [0,0,0]
    velocity = [0,0,0]
    saturnInputPayload.r_BdyZero_N = position
    saturnInputPayload.v_BdyZero_N = velocity
    saturnInputPayload.timeTag = 1234.0
    saturnInputMessage = messaging.EphemerisMsgF32().write(saturnInputPayload)
    saturnBody.inputEphemerisMsg.subscribeTo(saturnInputMessage)
    saturnBody.bodySpiceId = SATURN_ID
    saturnBody.originalCentralBodyId = SATURN_ID
    list_of_bodies.append(saturnBody)

    dataLogList = list()
    names = []
    for body in list_of_bodies:
        ephemRecenter.addBodyEphemerisToRecenter(body)
        names.append(body.bodySpiceId)
    for body in list_of_bodies:
        index = ephemRecenter.getBodyIndexFromId(body.bodySpiceId)
        rec = ephemRecenter.recenteredEphemerisOutputMsgs[index].recorder()
        dataLogList.append(rec)
        unitTestSim.AddModelToTask(unitTaskName, rec)

    ephemRecenter.setPreviousCommonZeroBase(SATURN_ID)
    ephemRecenter.setNewZeroBase(MOON_ID)
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(0)
    unitTestSim.ExecuteSimulation()

    moon_r_recentered = np.array(dataLogList[3].r_BdyZero_N).reshape([3,])
    moon_v_recentered = np.array(dataLogList[3].v_BdyZero_N).reshape([3,])
    moon_time_tag = dataLogList[3].timeTag

    np.testing.assert_array_almost_equal(moon_r_recentered, np.zeros(3), 15,"Moon position error")
    np.testing.assert_array_almost_equal(moon_v_recentered, np.zeros(3), 15, "Moon velocity error")
    np.testing.assert_equal(moon_time_tag, moonInputPayload.timeTag, "Moon time tag error")

    earth_r_recentered = np.array(dataLogList[1].r_BdyZero_N).reshape([3,])
    earth_v_recentered = np.array(dataLogList[1].v_BdyZero_N).reshape([3,])
    earth_time_tag = dataLogList[1].timeTag

    np.testing.assert_array_almost_equal(earth_r_recentered,
                                  - np.array(moonInputPayload.r_BdyZero_N),
                                         15,
                                  "Earth position error")
    np.testing.assert_array_almost_equal(earth_v_recentered,
                                  - np.array(moonInputPayload.v_BdyZero_N),
                                         15,
                                  "Earth velocity error")
    np.testing.assert_array_almost_equal(earth_time_tag, earthInputPayload.timeTag, 15, "Earth time tag error")

    sun_r_recentered = np.array(dataLogList[0].r_BdyZero_N).reshape([3,])
    sun_v_recentered = np.array(dataLogList[0].v_BdyZero_N).reshape([3,])
    sun_time_tag = dataLogList[0].timeTag

    np.testing.assert_array_almost_equal(sun_r_recentered,
                                  np.array(sunInputPayload.r_BdyZero_N) -
                                  (np.array(moonInputPayload.r_BdyZero_N) + np.array(earthInputPayload.r_BdyZero_N)),
                                         15,
                                  "Sun position error")
    np.testing.assert_array_almost_equal(sun_v_recentered,
                                  np.array(sunInputPayload.v_BdyZero_N) -
                                  (np.array(moonInputPayload.v_BdyZero_N) + np.array(earthInputPayload.v_BdyZero_N)),
                                         15,
                                  "Sun velocity error")
    np.testing.assert_equal(sun_time_tag, sunInputPayload.timeTag, "Sun time tag error")

    saturn_r_recentered = np.array(dataLogList[4].r_BdyZero_N).reshape([3,])
    saturn_v_recentered = np.array(dataLogList[4].v_BdyZero_N).reshape([3,])
    saturn_time_tag = dataLogList[3].timeTag

    np.testing.assert_array_almost_equal(saturn_r_recentered,
                                  np.array(saturnInputPayload.r_BdyZero_N) -
                                  (np.array(moonInputPayload.r_BdyZero_N) + np.array(earthInputPayload.r_BdyZero_N)),
                                         15,
                                  "Saturn position error")
    np.testing.assert_array_almost_equal(saturn_v_recentered,
                                  np.array(saturnInputPayload.v_BdyZero_N) -
                                  (np.array(moonInputPayload.v_BdyZero_N) + np.array(earthInputPayload.v_BdyZero_N)),
                                         15,
                                  "Saturn velocity error")
    np.testing.assert_equal(saturn_time_tag, saturnInputPayload.timeTag, "Saturn time tag error")

def clearing_values():
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName,  int(0.5e9)))

    ephemRecenter = ephemeridesRecenterF32.EphemeridesRecenter()
    ephemRecenter.modelTag = "ephemeridesRecenter"
    unitTestSim.AddModelToTask(unitTaskName, ephemRecenter)

    list_of_bodies = []
    # Create bodies to add to the module
    sunBody = ephemeridesRecenterF32.BodyEphemeris()
    sunInputPayload = messaging.EphemerisMsgF32Payload()
    position = [0,0,0]
    velocity = [0,0,0]
    sunInputPayload.r_BdyZero_N = position
    sunInputPayload.v_BdyZero_N = velocity
    sunInputPayload.timeTag = 1234.0
    sunInputMessage = messaging.EphemerisMsgF32().write(sunInputPayload)
    sunBody.inputEphemerisMsg.subscribeTo(sunInputMessage)
    sunBody.bodySpiceId = SUN_ID
    sunBody.originalCentralBodyId = SUN_ID
    list_of_bodies.append(sunBody)

    # Set this message
    earthBody = ephemeridesRecenterF32.BodyEphemeris()
    earthInputPayload = messaging.EphemerisMsgF32Payload()
    position = [1000, -200, 100]
    velocity = [10, 0, -8]
    earthInputPayload.r_BdyZero_N = position
    earthInputPayload.v_BdyZero_N = velocity
    earthInputPayload.timeTag = 1234.0
    earthInputMessage = messaging.EphemerisMsgF32().write(earthInputPayload)
    earthBody.inputEphemerisMsg.subscribeTo(earthInputMessage)
    earthBody.bodySpiceId = EARTH_ID
    earthBody.originalCentralBodyId = SUN_ID
    list_of_bodies.append(earthBody)

    marsBody = ephemeridesRecenterF32.BodyEphemeris()
    marsInputPayload = messaging.EphemerisMsgF32Payload()
    position = [-4000, 3000, 10000]
    velocity = [-1, -2, 1]
    marsInputPayload.r_BdyZero_N = position
    marsInputPayload.v_BdyZero_N = velocity
    marsInputPayload.timeTag = 1234.0
    marsInputMessage = messaging.EphemerisMsgF32().write(marsInputPayload)
    marsBody.inputEphemerisMsg.subscribeTo(marsInputMessage)
    marsBody.bodySpiceId = MARS_ID
    marsBody.originalCentralBodyId = SUN_ID
    list_of_bodies.append(marsBody)

    moonBody = ephemeridesRecenterF32.BodyEphemeris()
    moonInputPayload = messaging.EphemerisMsgF32Payload()
    position = [-50, 30, 100]
    velocity = [-0.5, -0.2, 0.1]
    moonInputPayload.r_BdyZero_N = position
    moonInputPayload.v_BdyZero_N = velocity
    moonInputPayload.timeTag = 1234.0
    moonInputMessage = messaging.EphemerisMsgF32().write(moonInputPayload)
    moonBody.inputEphemerisMsg.subscribeTo(moonInputMessage)
    moonBody.bodySpiceId = MOON_ID
    moonBody.originalCentralBodyId = EARTH_ID
    list_of_bodies.append(moonBody)

    names = []
    for body in list_of_bodies:
        ephemRecenter.addBodyEphemerisToRecenter(body)
        names.append(body.bodySpiceId)

    ephemRecenter.setPreviousCommonZeroBase(SUN_ID)
    ephemRecenter.setNewZeroBase(MARS_ID)

    np.testing.assert_equal(ephemRecenter.getNumberOfBodies(), len(list_of_bodies), "Wrong number of bodies")
    np.testing.assert_equal(list(ephemRecenter.getAllIds()[:ephemRecenter.getNumberOfBodies()]), names, "Wrong IDs for bodies")

    np.testing.assert_equal(ephemRecenter.getNewZeroBase(), MARS_ID, "Wrong new zero base")
    np.testing.assert_equal(ephemRecenter.getPreviousCommonZeroBase(), SUN_ID, "Wrong previous zero base")

    ephemRecenter.clearAllBodies()
    np.testing.assert_equal(ephemRecenter.getNumberOfBodies(), 0, "Clear bodies did no work")



if __name__ == '__main__':
    mars_central_body()
