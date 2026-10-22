#!/usr/bin/env python3
"""The colour-match SDL simulator on ivoryos-next with Beaver's Bench, the 3D deck panel.

Needs ivoryos-next's edge server installed in the same environment. The port comes from
IVORYOS_PORT (default 8080), which the IvoryOS desktop launcher sets for a script profile.
"""

from scripts.sim_instruments import SimWeighBalance
from scripts.sim_instruments import SimRoboticArm
from scripts.sim_instruments import SimStirPlate
from scripts.sim_instruments import SimSolidAdditionStation
from scripts.sim_instruments import SimSampleAnalysisStation
from scripts.sim_instruments import SimCappingStation
from scripts.sim_instruments import SimLiquidAdditionStation
from beavers_bench.next_plugin import plugin
import ivoryos_edge

# Initialize hardware
try:
    weigh_balance = SimWeighBalance()
    robotic_arm = SimRoboticArm()
    stir_plate = SimStirPlate()
    solid_addition_station = SimSolidAdditionStation()
    sample_analysis_station = SimSampleAnalysisStation()
    capping_station = SimCappingStation()
    liquid_addition_station = SimLiquidAdditionStation()
except Exception as e:
    print(f"Failed to initialize hardware: {e}. Connect them in the web interface or try again.")

# Start the IvoryOS web interface
if __name__ == "__main__":
    ivoryos_edge.run(__name__, plugins=[plugin])
