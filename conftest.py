import inspect
import os

import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
print(path)


def pytest_addoption(parser):
    parser.addoption("--show_plots", action="store_true",
                     help="test(s) shall display plots")


@pytest.fixture(scope="module")
def show_plots(request):
    return request.config.getoption("--show_plots")


def pytest_make_parametrize_id(config, val, argname):
    return f"{argname}={val}"
