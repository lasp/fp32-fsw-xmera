import inspect
import os

import numpy as np

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

from xmera.utilities import SimulationBaseClass
from xmera.fp32 import inertial3DF32
from xmera.utilities import macros
from xmera.architecture import messaging


def test_set_reference():
    # Randomize the input to inertial 3D. MRPs can be a set of 3 numbers where those 3 numbers can be any real numbers
    rng = np.random.default_rng()
    sigma_input_RN = rng.normal(loc=0.0, scale=1e9, size=3)

    # Construct algorithm and associated C++ container
    module = inertial3DF32.Inertial3D()
    module.modelTag = "inertial3D"

    module.sigma_RN = sigma_input_RN

    run_test(module, sigma_input_RN)

def test_unset_reference():
    sigma_input_RN = [0.0, 0.0, 0.0]

    # Construct algorithm and associated C++ container
    module = inertial3DF32.Inertial3D()
    module.modelTag = "inertial3D"

    run_test(module, sigma_input_RN)

def run_test(module, sigma_input_RN):
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"

    # Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(0.5)     # update process rate update time
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, module)

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = module.attRefOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, dataLog)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(1.))        # seconds to stop simulation
    unit_test_sim.ExecuteSimulation()

    # retrieve the module output
    sigma_RN = dataLog.sigma_RN
    omega_RN_N = dataLog.omega_RN_N
    domega_RN_N = dataLog.domega_RN_N

    # set the filtered output truth states
    sigma_truth_RN = [sigma_input_RN] * 3
    omega_truth_RN_N = [[0.0, 0.0, 0.0]] * 3
    domega_truth_RN_N = [[0.0, 0.0, 0.0]] * 3

    # compare the module results to the truth values
    accuracy = 1e-7

    # Test the getter method
    np.testing.assert_allclose(np.array(module.sigma_RN).flatten(), sigma_input_RN, rtol=accuracy, verbose=True)

    # Test the outputs of the module
    np.testing.assert_allclose(sigma_RN, sigma_truth_RN, rtol=accuracy, verbose=True)
    np.testing.assert_allclose(omega_RN_N, omega_truth_RN_N, rtol=accuracy, verbose=True)
    np.testing.assert_allclose(domega_RN_N, domega_truth_RN_N, rtol=accuracy, verbose=True)


if __name__ == "__main__":
    test_set_reference()
