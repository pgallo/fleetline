import pytest

from fleetline.simulator import FleetSimulator, SimConfig


@pytest.fixture(scope="session")
def small_sim() -> FleetSimulator:
    return FleetSimulator(SimConfig(vehicles=4, hours=4.0, seed=11))


@pytest.fixture(scope="session")
def small_points(small_sim):
    return small_sim.run()
