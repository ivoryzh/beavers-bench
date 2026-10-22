// From one instrument-level call to what the arm does: the ladder the onboarding course draws
// (operation -> ArmManager -> UR3Arm). Every pick and place follows the routine the location file
// implies: home for that side of the deck, the safe point, the approach point above the grab,
// straight down to the grab, gripper, and back out the same way.
import { DECK, isTraySlot, slotLocation, type DeckDef, type DeckPosition, type Location, type Pose } from './deck';
import { chain, rotZ, translate, type Mat4 } from './mat4';
import type { SimEvent } from './simulator';

export type Held = 'vial' | 'tlh' | 'head' | 'cap';

export type Step =
  | { kind: 'movej'; name: string; code: string; target: Mat4 | DeckPosition }
  | { kind: 'movel'; name: string; code: string; target: Mat4 }
  | { kind: 'gripper'; code: string; open: boolean; width: number }
  | { kind: 'attach'; code: string; what: Held }
  | { kind: 'detach'; code: string; what: Held; at: string; slot?: number }
  | { kind: 'station'; code: string; station: string; action: string; args: unknown[]; seconds: number }
  | { kind: 'mix'; code: string; cycles: number; seconds: number; tilt: number } // tilt in radians, each way
  | { kind: 'wait'; code: string; seconds: number };

export interface Choreography {
  call: string;
  steps: Step[];
  note?: string; // why there is nothing (or little) to show
}

const deg = (d: number) => (d * Math.PI) / 180;

/** The TCP transform of a file pose. */
export function tcpPose(p: Pose): Mat4 {
  return chain(translate(p.x, p.y, p.z), rotZ(deg(p.rz)));
}

/** A pose moved by an offset given in the TCP's own frame. */
function offset(p: Pose, o: { x?: number; y?: number; z?: number }): Pose {
  const c = Math.cos(deg(p.rz)), s = Math.sin(deg(p.rz));
  const ox = o.x ?? 0, oy = o.y ?? 0, oz = o.z ?? 0;
  return { x: p.x + c * ox - s * oy, y: p.y + s * ox + c * oy, z: p.z + oz, rz: p.rz };
}

export const approachPose = (l: Location): Pose => ({ ...l.grab, z: l.grab.z + l.approach });
export const safePose = (l: Location): Pose => offset(approachPose(l), l.safe);
const bufferSafePose = (l: Location): Pose => offset(approachPose(l), l.bufferSafe ?? l.safe);

const fmtArgs = (args: unknown[]) => args.map((a) => (typeof a === 'number' ? +a.toFixed(3) : JSON.stringify(a))).join(', ');

const home = (side: DeckPosition): Step => ({ kind: 'movej', name: `home_${side}`, code: `arm.move_home('${side}')`, target: side });
const movej = (name: string, p: Pose): Step => ({ kind: 'movej', name, code: `arm.move_to_location('${name}')`, target: tcpPose(p) });
const movel = (name: string, p: Pose): Step => ({ kind: 'movel', name, code: `arm.move_to_location('${name}', linear=True)`, target: tcpPose(p) });
const open = (width: number, why = ''): Step => ({ kind: 'gripper', code: `gripper.open(${width})${why}`, open: true, width });
const close = (why = ''): Step => ({ kind: 'gripper', code: `gripper.close()${why}`, open: false, width: 0 });
const door = (station: string, action: 'door_open' | 'door_close'): Step =>
  ({ kind: 'station', code: `${station === 'weigh_balance' ? 'quantos' : station}.${action}()`, station, action, args: [], seconds: 0.7 });

/** Into a location and back out: safe -> approach -> grab, then `inside`, then approach -> safe. */
function visit(name: string, l: Location, inside: Step[]): Step[] {
  const steps: Step[] = [home(l.deckPosition)];
  if (l.door) steps.push(door(l.door, 'door_open'));
  if (l.bufferSafe) steps.push(movej(`${name}_buffer_safe`, bufferSafePose(l)), movel(`${name}_safe`, safePose(l)));
  else steps.push(movej(`${name}_safe`, safePose(l)));
  steps.push(movel(`${name}_approach`, approachPose(l)));
  steps.push(...inside);
  steps.push(movel(`${name}_approach`, approachPose(l)), movel(`${name}_safe`, safePose(l)));
  if (l.bufferSafe) steps.push(movel(`${name}_buffer_safe`, bufferSafePose(l)));
  if (l.door) steps.push(door(l.door, 'door_close'));
  steps.push(home(l.deckPosition));
  return steps;
}

function pick(name: string, l: Location, what: Held): Step[] {
  return visit(name, l, [
    open(l.gripperOpen),
    movel(name, l.grab),
    close('  # then check something is between the fingers'),
    { kind: 'attach', code: '', what },
  ]);
}

function place(name: string, l: Location, what: Held, at: string, slot?: number): Step[] {
  return visit(name, l, [
    movel(name, l.grab),
    open(l.gripperOpen),
    { kind: 'detach', code: '', what, at, slot },
  ]);
}

const stationOfArmAction = (action: string): string | null => {
  const m = action.match(/^(?:pick_up|place)_vial_(?:on|in)_(.+)$/);
  return m ? m[1] : null;
};

/**
 * The arm legs for one event. `slotHint` is the slot of the vial the arm is holding or has just put
 * down, for events (place_vial_in_tray with no argument) that do not say which one.
 */
export function choreograph(event: SimEvent, slotHint: number | null, deck: DeckDef = DECK): Choreography {
  const { instrument, action, args } = event;
  const call = `${instrument}.${action}(${fmtArgs(args ?? [])})`;
  const steps: Step[] = [];
  let note: string | undefined;

  if (instrument === 'robotic_arm') {
    if (action === 'pick_up_vial_from_tray') {
      const n = isTraySlot(Number(args?.[0])) ? Number(args[0]) : slotHint;
      if (n && isTraySlot(n)) steps.push(...pick(`tray[${n}]`, slotLocation(n, deck.tray), 'vial'));
    } else if (action === 'place_vial_in_tray') {
      const n = isTraySlot(Number(args?.[0])) ? Number(args[0]) : slotHint;
      if (n && isTraySlot(n)) steps.push(...place(`tray[${n}]`, slotLocation(n, deck.tray), 'vial', 'tray', n));
    } else {
      const station = stationOfArmAction(action);
      const def = station ? deck.stations[station] : undefined;
      if (def?.hold) {
        // the arm keeps the vial and shows it to the camera
        const l = def.location;
        if (action.startsWith('pick_up')) {
          steps.push(home(l.deckPosition));
        } else {
          note = 'the arm keeps the vial and holds it up to the camera';
          steps.push(home(l.deckPosition), movej(def.id, l.grab), { kind: 'detach', code: '', what: 'vial', at: def.id });
        }
      } else if (def?.virtual) {
        note = `this deck has no ${def.id.replace(/_/g, ' ')}: the vial stays in the gripper`;
        steps.push({ kind: 'detach', code: '', what: 'vial', at: def.id }); // bookkeeping only: the vial is "at" the station
        if (action.startsWith('pick_up')) steps.length = 0;
      } else if (def) {
        if (action.startsWith('pick_up')) steps.push(...pick(def.id, def.location, 'vial'));
        else steps.push(...place(def.id, def.location, 'vial', def.id));
      }
    }
  } else if (instrument === 'liquid_addition_station' && action === 'add_liquid') {
    // tlh_at_base_station -> tlh_at_capping_station (relative to the vial) -> dose -> back
    const vialGrab = deck.stations.liquid_addition_station.location.grab;
    const [ox, oy, oz] = deck.tlh.doseOffset;
    const dose: Location = {
      grab: { x: vialGrab.x + ox, y: vialGrab.y + oy, z: vialGrab.z + oz, rz: deck.tlh.base.grab.rz },
      approach: deck.tlh.doseApproach, safe: {}, deckPosition: deck.tlh.base.deckPosition, gripperOpen: deck.tlh.base.gripperOpen,
    };
    steps.push(...pick('tlh_wash_station', deck.tlh.base, 'tlh'));
    steps.push(
      home(dose.deckPosition),
      movej('tlh_at_capping_station_approach', approachPose(dose)),
      movel('tlh_at_capping_station', dose.grab),
      { kind: 'station', code: `tlh.dispense(${fmtArgs(args ?? [])})`, station: 'liquid_addition_station', action, args: args ?? [], seconds: 1.8 },
      movel('tlh_at_capping_station_approach', approachPose(dose)),
    );
    steps.push(...place('tlh_wash_station', deck.tlh.base, 'tlh', 'tlh_base'));
  } else if (instrument === 'solid_addition_station' && action === 'add_solid') {
    if (deck.dosingHeads) {
      // dosing_head_at_rack -> dosing_head_at_quantos -> dose -> back to the rack
      const rackSlot = deck.dosingHeads.rack[0];
      steps.push(...pick('dosing_head_1', rackSlot, 'head'));
      steps.push(...place('dosing_head_at_quantos', deck.dosingHeads.quantos, 'head', 'quantos_head'));
      steps.push({ kind: 'station', code: `quantos.dose(${fmtArgs(args ?? [])})`, station: 'solid_addition_station', action, args: args ?? [], seconds: 3.0 }); // the head rides down to the vial, powder, back up
      steps.push(...pick('dosing_head_at_quantos', deck.dosingHeads.quantos, 'head'));
      steps.push(...place('dosing_head_1', rackSlot, 'head', 'head_rack_1'));
    } else {
      note = 'no solid dosing on this bench';
      steps.push({ kind: 'station', code: `# no doser here: ${call}`, station: 'solid_addition_station', action, args: args ?? [], seconds: 0.6 });
    }
  } else if (instrument === 'capping_station' && (action === 'uncap_vial' || action === 'cap_vial')) {
    // the arm holds the cap still while the capper's jaws grip and spin the vial; the cap waits in the cap holder
    const vial = deck.stations.capping_station.location;
    const capAt = (dz: number): Location => ({ ...vial, grab: { ...vial.grab, z: vial.grab.z + dz }, approach: 0.05, gripperOpen: 60 });
    if (action === 'uncap_vial') {
      const l = capAt(deck.capping.uncapZ);
      steps.push(...visit('capping_station_cap', l, [
        open(l.gripperOpen),
        movel('capping_station_cap', l.grab),
        close('  # on the cap'),
        { kind: 'attach', code: '', what: 'cap' },
        { kind: 'station', code: 'capper.grip_and_spin(counterclockwise)', station: 'capping_station', action, args: args ?? [], seconds: deck.capping.spinSeconds },
      ]));
      steps.push(...place('cap_holder', deck.capHolder, 'cap', 'cap_holder'));
    } else {
      const l = capAt(deck.capping.capZ);
      steps.push(...pick('cap_holder', deck.capHolder, 'cap'));
      steps.push(...visit('capping_station_cap', l, [
        movel('capping_station_cap', l.grab),
        { kind: 'station', code: 'capper.grip_and_spin(clockwise)', station: 'capping_station', action, args: args ?? [], seconds: deck.capping.spinSeconds },
        open(l.gripperOpen),
        { kind: 'detach', code: '', what: 'cap', at: 'vial' },
      ]));
    }
  } else if (instrument === 'weigh_balance') {
    const dev = deck.balance === 'quantos' ? 'quantos' : 'balance';
    steps.push({ kind: 'station', code: action === 'zero' ? `${dev}.zero()` : `${dev}.weigh()`, station: 'weigh_balance', action, args: args ?? [], seconds: 0.8 });
  } else if (instrument === 'stir_plate') {
    if (action === 'start_stirring') {
      note = 'no stir plate: the arm mixes by rocking the vial';
      steps.push({ kind: 'mix', code: `arm.rock_vial(times=${deck.mixing.cycles}, degrees=${deck.mixing.tiltDeg})`, cycles: deck.mixing.cycles, seconds: deck.mixing.seconds, tilt: deg(deck.mixing.tiltDeg) });
    } else {
      note = 'mixing already done by the arm';
    }
  } else if (instrument === 'sample_analysis_station') {
    note = 'the webcam takes a picture of the vial the arm holds up';
    steps.push({ kind: 'station', code: 'camera.capture()', station: 'analysis_station', action, args: args ?? [], seconds: 1.2 });
  }

  return { call, steps, note };
}
