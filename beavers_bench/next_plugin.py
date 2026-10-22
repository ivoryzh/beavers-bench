"""ivoryos-next plugin for Beaver's Bench, the 3D deck simulation.

    import ivoryos_edge
    from beavers_bench.next_plugin import plugin

    ivoryos_edge.run(__name__, plugins=[plugin])

Same wiring as sdl_sim_web.next_plugin: the simulator's events go through the plugin bridge to the
page (static/, built from frontend/ with `npm run build`), and the deck state is pushed after every
instrument call. The page expands each arm call into the moves the real UR3 ArmManager makes.
"""
from ivoryos_edge.plugins import Plugin

from plugin_bridge import PluginBridge
from scripts.sim_instruments import build_sim_state, configure_sim_plugin

plugin = Plugin("Beaver's Bench", id="beavers_bench", page="static", placement="panel-left", icon="box")
bridge = PluginBridge()
bridge.bind(plugin)


@plugin.on_start
def _start(instruments):
    configure_sim_plugin(bridge.process_event, bridge, monitor=False)
    bridge.watch_state(plugin, build_sim_state)
