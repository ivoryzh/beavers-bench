# Beaver's Bench frontend

The 3D deck simulation, a Vite + TypeScript + three.js page. `npm run build` writes the page to
`../beavers_bench/static`, which the Python package ships and `beavers_bench/next_plugin.py` serves
inside ivoryos-next. The same build opens from any static file server, or `file://`, in **sandbox mode**:
the simulator then runs in the page (`src/simulator.ts`, a port of `scripts/instruments.py` with
the same method names, checks and error messages) and every instrument method is a button.

```bash
npm install
npm run dev      # http://localhost:5173, sandbox mode, hot reload
npm test         # kinematics against the URDF joint table, IK round trips
npm run build    # type-check, then write ../beavers_bench/static (commit the result)
```

## How it works

| File | What it holds |
| --- | --- |
| `src/deck.ts` | **The deck as data, from the real location file** (`deck/locations_education_deck.yaml`, in millimetres; the TypeScript copy is in metres): the tray's three corner poses, the capping station, the Quantos, the TLH base station, the dosing-head rack, the three home poses (joints), the gripper's TCP, and each location's approach and safe offsets. Items the file does not have are marked `estimated` or `placeholder`. |
| `src/kinematics.ts` | Closed-form UR forward and inverse kinematics (all eight solutions), checked against `ur3.urdf` by the tests. `UR3E` parameters are there for an e-series. |
| `src/choreography.ts` | One instrument call → the arm's legs, the routine the location file implies: `home for that side of the deck → <name>_safe (movej) → <name>_approach (movel) → <name> (movel) → gripper → back out`. `add_liquid` fetches the TLH from its base station and doses the vial where it stands in the capping station; `add_solid` carries a dosing head from the rack into the Quantos and back. |
| `src/animator.ts` | Plays the legs: joint-space interpolation for movej, straight lines with IK every frame for movel, the gripper, station actions. Speeds up when the Python deck runs ahead. |
| `src/scene.ts` | three.js: table, stations, tray and vials, the URDF arm (meshes fetched from the ros-industrial repository through jsDelivr), gripper, liquid handler, labels, safe-height points, tool path. |
| `src/events.ts` | Live mode (the plugin bridge, when `bridge-client.js` is served next to the page) or sandbox mode. |
| `src/ui.ts`, `index.html`, `src/style.css` | The panel: the call being played and its ladder of moves, deck state, view toggles, sandbox buttons, log. Colours follow the `dark` class the bridge puts on `<html>` (the IvoryOS theme); standalone, the operating system. |

## What comes from the location file, and what does not

`src/deck.ts` is a hand conversion of `deck/locations_education_deck.yaml` (keep both in step):

- **From the file**: `tray` is `tray_2` (5 × 3, 16 mL vials, the simulator's 15 slots; slots 1–5 run
  from `grab` toward `bottom_left`), `otherTrays` are `tray_1` and `tray_3`, `capping_station` is
  `vial_at_capping_station.vial_16_ml`, the Quantos is `vial_at_quantos.vial_16_ml`, the TLH is
  `tlh_at_base_station` plus the `tlh_at_capping_station` offset, the dosing heads are
  `dosing_head_at_rack` and `dosing_head_at_quantos`, `homes` are `arm_home`, `tcp` is
  `grippers.universal_gripper.tcp`, and `gripperOpen` values are `gripper_positions`.
- **Two simulator stations share one instrument**: with the Quantos, `weigh_balance` and
  `solid_addition_station` are both the Quantos; on either bench `liquid_addition_station` is the capping
  station (the TLH doses the uncapped vial there, after the arm fetches it from its wash station). The
  simulator still moves the vial between them, so the arm lifts and re-seats it in place. The Quantos
  door slides open for every visit (`door` on a location).
- **Estimated**: the table and the robot's plate, the plain balance of the second bench, the webcam,
  and the Quantos' outer shape (its side door, on the robot's side, slides toward the back).
- **Only `tray_2` is drawn.** The file's `tray_1`, `tray_3`, `tray_4` and `tray_5` are listed in the
  "Locations only" view but not on the bench.
- **The Quantos is not at the file's `vial_at_quantos`.** It stands in the table's far corner, its
  dosing-head hook on the file's `dosing_head_at_quantos` point slid `QUANTOS_SHIFT` (2 cm out, 4 cm
  back: as far as the arm can still dock the head) so the capping station is clear of its side door. The
  vial holder is in the middle of the chamber, right under the hook (estimated). The file's `vial_at_quantos`
  and `dosing_head_at_quantos` points are still shown in "Locations only".
- **Vials go into the Quantos sideways, the way the lab loads it.** The arm comes down outside the
  balance (`weigh_balance_buffer_safe`, high, then `weigh_balance_safe` at the approach height, 7 cm clear
  of the door), slides straight in over the holder, drops 4 cm to seat the vial, lifts, slides straight
  back out, and only then does the door close. The vial is gripped like a tray vial (rz -90).
- **The Quantos head hangs in a clip on a linear rail.** The head is the photographed one: a clear
  cartridge, a white cap, a steel collar and nozzle at the bottom. The gripper takes it between its pads,
  not its fingertips, so it hangs `GRIPPER_PADS` behind and above the file's TCP point, and the vial
  holder is under that. A silver clip on an arm from a carriage on a wide white rail up the back of the
  chamber holds it by the collar, upright, exactly where the robot drops it. `add_solid` plays
  `quantos.dose()`: the head rides straight down until its nozzle is at the vial mouth, the powder flows,
  and it rides back up. The chamber's top glass rides on the same carriage, just under the head's collar
  with the nozzle through it: the chamber walls reach up to it, so it closes the chamber while the head
  is parked, and it comes down into the chamber with the head to dose. There is no front window and no
  module box.
- **The gripper** is a thin two-jaw gripper hung under a black bracket, lying along the bracket with its
  pads facing each other at the jaw tips and four thin fingers (two per jaw) hanging straight down to the TCP.
  It grabs either way: vials from above between the fingertips, the dosing head sideways between the
  pads. The jaws open only a little wider than what they hold, never flat out, and nothing stands proud
  of the gripper, so it fits through the balance door. The 16 mL vial is drawn 20 mm across.
- **The tray is slid `TRAY_SHIFT` (15 cm along the rack)** so its vial tops are clear of the dosing-head
  rack's lower row (the file's slots sit right under it), and its slots are spread to `TRAY_PITCH` (32 mm,
  the file's 25 mm leaves no room for a finger between two vials). The rack is at the UR3e model's reach
  and cannot move away; the tray cannot come toward the robot (its safe points go out of reach within 2 cm).
- **Nothing guessed is drawn any more**: the solvent tray and the VICI valve are gone. The webcam stays,
  because `place_vial_in_analysis_station` holds the vial up to it.
- **The arm is a UR3e**: `public/ur3e.urdf` is `ur3.urdf` with the e-series joint origins and visual
  offsets from ros-industrial's `ur_description` (`config/ur3e/*.yaml`); the meshes come from the same
  repository through the CDN. The file's `positive_x` home pose lands within 1.5 cm of the model.
- **Two benches, one switch.** The Bench selector (saved in the browser, the page reloads) picks
  `buildDeck('quantos')`, the location file's bench, or `buildDeck('balance')`, the bench with a plain
  weigh balance and no solid dosing (its balance position is estimated). On the second, `add_solid`
  shows the call and does nothing else.
- **No stir plate.** The simulator's calls still run: the vial "at" the stir plate stays in the gripper
  and `start_stirring` makes the arm rock it (wrist 2 tilts it 30 degrees one way, then the other, three
  times: `mixing` in `deck.ts`), which is how the lab mixes. It is `virtual` in `deck.ts`, so it can be given a real location later.
- **The camera is the analysis station.** `place_vial_in_analysis_station` has the arm hold the vial up
  in front of the webcam (a `hold` station: no release), `analyze` flashes the lens, and
  `pick_up_vial_in_analysis_station` just goes home. The webcam's position is estimated from the photos.
- **Capping is a two-party move.** The capper is a white box with two white blocks on a turntable and
  nothing above the vial. For `uncap_vial` the arm goes to the capped vial, closes on the cap
  `uncapping_z_offset` above the vial's grab pose, the blocks close on the vial and turn with it, spinning
  the vial out of the cap the arm holds still, and the arm carries the cap to the file's
  `vial_cap_at_cap_holder` location. `cap_vial` fetches it back and the blocks turn the other way.
- **The TLH** is a gripped block with a needle whose tip is 55 mm below the TCP, parked at its wash
  station (the station block behind it, away from the robot, with tubing out of the bottom). The file
  doses with the TCP 50 mm above the vial's grab pose, so the tip goes 38 mm into the vial and clears
  its top at the approach point (checked by a test).
- **Reach.** The dosing-head rack sits a few millimetres past the UR3e model's reach (the TCP and the
  arm's true lengths differ by about that); `ikNudged` pulls such targets in and the log says by how much.
