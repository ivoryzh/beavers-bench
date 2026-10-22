"""Beaver's Bench served the way ivoryos-next serves it (see ivoryos-next docs/plugins.md)."""
import importlib.util

import pytest

pytestmark = pytest.mark.skipif(
    importlib.util.find_spec("ivoryos_edge") is None, reason="ivoryos-next edge server not installed"
)


@pytest.fixture
def bench_deck():
    from fastapi import FastAPI
    from fastapi.testclient import TestClient
    from ivoryos_edge.plugins import start_plugin

    from scripts import sim_instruments
    from beavers_bench.next_plugin import plugin

    # the simulator as a new process has it: default state, no plugin or reset handler left over
    sim_instruments.reset_state()
    sim_instruments._plugin = None
    sim_instruments._reset_handler = None
    instruments = {
        "robotic_arm": sim_instruments.SimRoboticArm(),
        "capping_station": sim_instruments.SimCappingStation(),
    }
    app = FastAPI()
    assert start_plugin(app, plugin, instruments) is None
    with TestClient(app) as client:
        yield client, instruments


def test_serves_the_built_page_and_the_deck_state(bench_deck):
    """The same bridge as the 2D page, serving the Vite build in beavers_bench/static (a missing
    build would 404 here) and the simulator's events."""
    client, instruments = bench_deck
    page = client.get("/plugins/beavers_bench/")
    assert page.status_code == 200
    assert "Beaver" in page.text and 'type="module"' in page.text
    assert client.get("/plugins/beavers_bench/bridge-client.js").status_code == 200
    assert client.get("/plugins/beavers_bench/ur3e.urdf").status_code == 200

    instruments["robotic_arm"].pick_up_vial_from_tray(2)
    instruments["robotic_arm"].place_vial_on_capping_station()
    instruments["capping_station"].uncap_vial()
    state = client.get("/plugins/beavers_bench/api/snapshot").json()["state"]
    assert state["vial_location"] == "capping_station"
    assert state["current_vial"] == 2 and state["cap_is_on"] is False

    assert client.post("/plugins/beavers_bench/api/reset").status_code == 200
    assert client.get("/plugins/beavers_bench/api/snapshot").json()["state"]["vial_location"] == "tray"
