// The deck as data, from the real UR3e's location file (deck/locations_education_deck.yaml, the
// "KPMBA education deck"), converted from millimetres to metres. Poses are where the gripper's
// tool centre point (TCP) is when it holds the object at that location, in the robot's `base`
// frame (z up), with `rz` the yaw of the TCP in degrees as the file writes it: the TCP's z axis
// points up when the tool points down, because the gripper TCP is defined with rx = -180 deg.
//
// The routine every location implies, as the real ArmManager runs it:
//   home (for the location's side of the deck) -> safe -> approach (above the grab) -> grab
//   -> gripper -> approach -> safe -> home
// `approach` is a height above the grab pose; `safe` is an offset in the TCP's own frame from the
// approach point (a negative y is "back toward the robot", whichever way the TCP faces).
//
// Two benches share this file: one has a Quantos (powder dosing with its own balance), the other a
// plain weigh balance with a vial holder and no solid dosing. `buildDeck` picks one (`balance`).
// Things the file does not have are marked `estimated`; the simulator's stir plate and analysis
// station are `virtual`: this deck has neither, mixing is the arm inverting the vial it holds.
import type { Joints } from './kinematics';
import type { Vec3 } from './mat4';

export type DeckPosition = 'positive_x' | 'positive_y' | 'negative_y';
export type BalanceModule = 'quantos' | 'balance';

export interface Pose { x: number; y: number; z: number; rz: number } // metres, degrees

export interface Location {
  grab: Pose;
  approach: number;                           // metres above grab
  safe: { x?: number; y?: number; z?: number }; // TCP-frame offset from the approach point
  bufferSafe?: { x?: number; y?: number; z?: number }; // a second, further-out safe point (Quantos head)
  deckPosition: DeckPosition;
  gripperOpen: number;                        // mm, the opening the file names for this object
  door?: string;                              // a station whose door must open for this visit
}

export interface Footprint { w: number; d: number; h: number } // along x, along y, tall

export type StationDraw = 'box' | 'capper' | 'quantos' | 'balance' | 'none';

export interface StationDef {
  id: string;             // the simulator's name: robotic_arm.place_vial_on_<id>
  label: string;
  location: Location;     // where the simulator's vial is gripped at this station
  footprint: Footprint;   // standing on the table, centred under the vial (unless `center`)
  center?: [number, number];
  color: string;
  draw: StationDraw;
  sharesSeatWith?: string; // another station at the very same spot (one instrument, two roles)
  virtual?: boolean;       // not on this deck: the vial stays in the gripper, no arm legs
  hold?: boolean;          // the arm keeps the vial and presents it at `location` (the camera)
  estimated?: boolean;     // the position is a guess
  note?: string;
}

export interface TrayDef {
  label: string;
  grab: Pose;        // the file's reference slot: slot 1
  bottomLeft: Pose;  // last slot along the rows
  bottomRight: Pose; // the far corner
  rows: number;      // slots along grab -> bottomLeft
  cols: number;      // slots along bottomLeft -> bottomRight
  approach: number;
  safe: { x?: number; y?: number; z?: number };
  deckPosition: DeckPosition;
  gripperOpen: number;
  vial: VialDef;
}

export interface VialDef { radius: number; height: number; gripHeight: number; capColor: string }

export interface SceneryDef {
  id: string;
  label: string;
  center: Vec3;       // on the table surface
  footprint: Footprint;
  color: string;
  kind: 'box' | 'bottles' | 'cylinder';
  estimated?: boolean;
}

export interface DeckDef {
  name: string;
  source: string;
  arm: 'UR3' | 'UR3e';
  balance: BalanceModule;
  table: { x0: number; x1: number; y0: number; y1: number; top: number; thickness: number };
  mount: Footprint;
  tcp: { xyz: Vec3; rx: number };     // flange (tool0) -> TCP, as set on the robot
  homes: Record<DeckPosition, Joints>;
  gripper: { closed: number; open: number }; // mm
  vial: VialDef;                     // the 16 mL vial the simulator moves
  tray: TrayDef;                     // the simulator's 15-slot tray
  otherTrays: TrayDef[];             // racks on the deck the simulator does not use
  stations: Record<string, StationDef>;
  tlh: {                             // the mobile liquid handler ("TLH"): a needle the arm carries
    label: string;
    base: Location;                  // parked here, at its wash station, facing the robot
    doseOffset: Vec3;                // TCP offset from the vial's grab pose at the capping station
    doseApproach: number;
    needleTip: number;               // how far below the TCP the needle tip is when the TLH is held
  };
  capping: {                         // the capper: two jaws grip and spin the vial while the arm holds the cap
    uncapZ: number;                  // TCP above the vial's grab pose when taking the cap off
    capZ: number;                    // ... when screwing it on
    spinSeconds: number;
  };
  camera: { position: Vec3; label: string } | null; // a webcam the arm shows the vial to
  dosingHeads: {                     // Quantos powder-dosing heads (only with the Quantos)
    rack: Location[];
    quantos: Location;
  } | null;
  capHolder: Location;
  mixing: { cycles: number; seconds: number }; // the arm inverts the held vial this many times
  scenery: SceneryDef[];
}

const deg = (d: number) => (d * Math.PI) / 180;
const mm = (x: number, y: number, z: number, rz: number): Pose => ({ x: x / 1000, y: y / 1000, z: z / 1000, rz });

const TABLE_TOP = -0.015; // the robot stands on a thin Vention plate (estimated)

const VIAL_16_ML: VialDef = { radius: 0.0115, height: 0.062, gripHeight: 0.045, capColor: '#1f2937' };

const trayCommon = { approach: 0.09, safe: { y: -0.1, z: 0.05 }, deckPosition: 'positive_x' as DeckPosition };

const capping: Location = { grab: mm(157.0, -514.0, 205.9, 180), approach: 0.05, safe: { y: -0.15 }, deckPosition: 'negative_y', gripperOpen: 60 };
// The file's vial_at_quantos (502.1, -169.0, 191.0) is not where this Quantos stands: the bench has it at
// the far end of the capping side, its dosing-head hook on the file's dosing_head_at_quantos point and the
// balance pan directly under that hook (the head carrier lowers the gun onto the vial). The vial seat is
// therefore placed under the hook, estimated; the hook itself is the file's.
const QUANTOS_HOOK = mm(393.6, -355.8, 294.5, 180);
const quantosVial: Location = { grab: { x: QUANTOS_HOOK.x, y: QUANTOS_HOOK.y - 0.045, z: 0.135, rz: 180 }, approach: 0.04, safe: { y: -0.1 }, deckPosition: 'negative_y', gripperOpen: 40, door: 'weigh_balance' };
// the plain balance of the other bench: not in the file, placed where the photos show it, in front of the robot
const plainBalance: Location = { grab: { x: 0.40, y: -0.30, z: TABLE_TOP + 0.075 + 0.035 + VIAL_16_ML.gripHeight, rz: -90 }, approach: 0.09, safe: { y: -0.1, z: 0.05 }, deckPosition: 'positive_x', gripperOpen: 22 };
// stations the simulator has and this deck does not: the vial just stays in the gripper
const virtualLocation: Location = { grab: { x: 0, y: 0, z: 0, rz: 0 }, approach: 0, safe: {}, deckPosition: 'positive_x', gripperOpen: 22 };
// the webcam station of the photographed bench: the arm holds the vial in front of the lens (estimated)
const CAMERA_POSITION: Vec3 = [-0.24, -0.40, 0.19];
const cameraHold: Location = { grab: { x: -0.10, y: -0.38, z: 0.21, rz: 180 }, approach: 0, safe: {}, deckPosition: 'negative_y', gripperOpen: 22 };

const head = (x: number, y: number, z: number): Location => ({
  grab: mm(x, y, z, -90), approach: 0.0025, safe: { y: -0.1 }, deckPosition: 'positive_x', gripperOpen: 24,
});

const DOSING_HEADS = {
  rack: [
    head(585.4, 118.2, 353.5), head(586.6, 46.4, 353.5), head(586.0, 117.3, 190.2), head(586.3, 45.9, 190.2),
    head(586.2, -64.7, 338.5), head(586.1, -134.6, 337.3), head(585.2, -62.8, 228.8), head(586.7, -134.6, 230.1),
  ],
  quantos: { grab: QUANTOS_HOOK, approach: 0.002, safe: { y: -0.015 }, bufferSafe: { y: -0.07, z: 0.05 }, deckPosition: 'negative_y' as DeckPosition, gripperOpen: 24, door: 'weigh_balance' },
};

export function buildDeck(balance: BalanceModule = 'quantos'): DeckDef {
  const quantos = balance === 'quantos';
  return {
    name: "Beaver's Bench",
    source: 'deck/locations_education_deck.yaml',
    arm: 'UR3e',
    balance,
    table: { x0: -0.30, x1: 0.68, y0: -0.70, y1: 0.36, top: TABLE_TOP, thickness: 0.025 },
    mount: { w: 0.2, d: 0.2, h: -TABLE_TOP },
    tcp: { xyz: [0.0, -0.2, 0.052], rx: -Math.PI },
    homes: {
      positive_x: [155.04861363122606, -125.53948598140956, 81.39442541540217, -45.9412295629915, -89.67652155766287, 334.86147145692405 - 360].map(deg) as Joints,
      positive_y: [245.0, -125.54044220789336, 81.3932779436216, -45.938927789241205, -89.67474570847867, 334.8616627022208 - 360].map(deg) as Joints,
      negative_y: [60.00051393383986, -125.54044220789336, 81.3932779436216, -45.938927789241205, -89.67474570847867, 334.8616627022208 - 360].map(deg) as Joints,
    },
    gripper: { closed: 0, open: 110 },
    vial: VIAL_16_ML,
    tray: {
      label: 'Sample tray (tray_2, 16 mL)',
      grab: mm(547.2, 71.9, 91.5, -90),
      bottomLeft: mm(446.5, 71.37, 91.5, -90),
      bottomRight: mm(447.4, 22.94, 91.5, -90),
      rows: 5,
      cols: 3,
      ...trayCommon,
      gripperOpen: 22,
      vial: VIAL_16_ML,
    },
    otherTrays: [], // only tray_2 is on the bench; tray_1 and tray_3..5 of the file are not drawn
    stations: {
      capping_station: {
        id: 'capping_station', label: 'Capping station', location: capping,
        footprint: { w: 0.12, d: 0.12, h: 0.0 }, color: '#f3f4f6', draw: 'capper',
        note: 'vial_at_capping_station.vial_16_ml; the capper lifts the cap 25 mm.',
      },
      liquid_addition_station: {
        id: 'liquid_addition_station', label: 'Liquid addition: the TLH doses the vial here', location: capping,
        footprint: { w: 0, d: 0, h: 0 }, color: '#c7d2fe', draw: 'none', sharesSeatWith: 'capping_station',
        note: 'tlh_at_capping_station: the arm fetches the liquid handler from its wash station and doses the uncapped vial in the capping station.',
      },
      weigh_balance: quantos
        ? {
            id: 'weigh_balance', label: 'Quantos: balance + powder dosing', location: quantosVial, estimated: true,
            footprint: { w: 0.26, d: 0.33, h: 0.0 }, center: [QUANTOS_HOOK.x, QUANTOS_HOOK.y - 0.18], color: '#e5e7eb', draw: 'quantos',
            note: 'At the far end of the capping side, its dosing-head hook on the file\'s dosing_head_at_quantos point and the balance pan under it. The side door slides to the back for the arm. Vial seat estimated.',
          }
        : {
            id: 'weigh_balance', label: 'Weigh balance', location: plainBalance, estimated: true,
            footprint: { w: 0.20, d: 0.18, h: 0.075 }, color: '#e5e7eb', draw: 'balance',
            note: 'The bench without a Quantos: a balance with a vial holder on the pan. Not in the location file.',
          },
      solid_addition_station: quantos
        ? {
            id: 'solid_addition_station', label: 'Solid addition: a dosing head in the Quantos', location: quantosVial,
            footprint: { w: 0, d: 0, h: 0 }, color: '#fde68a', draw: 'none', sharesSeatWith: 'weigh_balance',
            note: 'dosing_head_at_rack -> dosing_head_at_quantos: the arm brings a dosing head over the vial.',
          }
        : {
            id: 'solid_addition_station', label: 'Solid addition', location: virtualLocation,
            footprint: { w: 0, d: 0, h: 0 }, color: '#fde68a', draw: 'none', virtual: true,
            note: 'No solid dosing on this bench: the simulator call is shown, the vial stays in the gripper.',
          },
      stir_plate: {
        id: 'stir_plate', label: 'Mixing (the arm inverts the vial)', location: virtualLocation,
        footprint: { w: 0, d: 0, h: 0 }, color: '#99f6e4', draw: 'none', virtual: true,
        note: 'No stir plate: mixing is the arm turning the vial it holds upside down and back.',
      },
      analysis_station: {
        id: 'analysis_station', label: 'Camera (webcam)', location: cameraHold, estimated: true,
        footprint: { w: 0, d: 0, h: 0 }, color: '#fbcfe8', draw: 'none', virtual: true, hold: true,
        note: 'The arm brings the vial in front of the webcam and holds it there for the picture. Position estimated from the photos.',
      },
    },
    tlh: {
      label: 'TLH wash station',
      base: { grab: mm(-43.9, -577.8, 212.2, -180), approach: 0.085, safe: { y: -0.15 }, deckPosition: 'negative_y', gripperOpen: 60 },
      doseOffset: [0, 0.0046, 0.05],
      doseApproach: 0.04,
      // the file doses with the TCP 50 mm above the vial's grab pose (the vial top is 17 mm above it),
      // so a tip 55 mm below the TCP reaches 38 mm into the vial and clears its top at the approach point
      needleTip: 0.055,
    },
    capping: { uncapZ: 0.015, capZ: 0.025, spinSeconds: 1.6 }, // uncapping_z_offset, capping_z_offset
    camera: { position: CAMERA_POSITION, label: 'Webcam' },
    dosingHeads: quantos ? DOSING_HEADS : null,
    capHolder: { grab: mm(23.8, -498.0, 152.2, 180), approach: 0.03, safe: { y: -0.1 }, deckPosition: 'negative_y', gripperOpen: 60 },
    mixing: { cycles: 2, seconds: 3.2 },
    scenery: [
      ...(quantos ? [] : [{ id: 'solvent_tray', label: 'Solvent tray', center: [0.40, -0.52, TABLE_TOP] as Vec3, footprint: { w: 0.32, d: 0.22, h: 0.06 }, color: '#16a34a', kind: 'bottles' as const, estimated: true }]),
      { id: 'vici_valve', label: 'VICI valve', center: quantos ? [0.08, -0.65, TABLE_TOP] : [0.18, -0.62, TABLE_TOP], footprint: { w: 0.08, d: 0.08, h: 0.11 }, color: '#1f2937', kind: 'box', estimated: true },
    ],
  };
}

export interface FilePoint { name: string; pose: Pose; group: 'home' | 'tray' | 'capping' | 'quantos' | 'heads' | 'tlh' }

/** Every engagement point the location file holds, by the file's own names (TCP poses, metres). */
export const FILE_POINTS: FilePoint[] = [
  { name: 'arm_home.positive_x', pose: mm(353.3, 78.9, 367.2, -90), group: 'home' },
  { name: 'arm_home.positive_y', pose: mm(-50.9, 358.4, 367.0, 0), group: 'home' },
  { name: 'arm_home.negative_y', pose: mm(47.5, -358.9, 367.1, 180), group: 'home' },
  { name: 'tray_1.grab', pose: mm(501.6, 190.0, 93, -90), group: 'tray' },
  { name: 'tray_1.bottom_left', pose: mm(401.1, 189.5, 93, -90), group: 'tray' },
  { name: 'tray_1.bottom_right', pose: mm(400.5, 129.6, 93, -90), group: 'tray' },
  { name: 'tray_2.grab', pose: mm(547.2, 71.9, 91.5, -90), group: 'tray' },
  { name: 'tray_2.bottom_left', pose: mm(446.5, 71.37, 91.5, -90), group: 'tray' },
  { name: 'tray_2.bottom_right', pose: mm(447.4, 22.94, 91.5, -90), group: 'tray' },
  { name: 'tray_3.grab', pose: mm(546.4, -41.26, 91.5, -90), group: 'tray' },
  { name: 'tray_3.bottom_left', pose: mm(446.6, -42.02, 91.5, -90), group: 'tray' },
  { name: 'tray_3.bottom_right', pose: mm(447.1, -91.85, 91.5, -90), group: 'tray' },
  { name: 'tray_4.grab', pose: mm(545.4, -144.4, 97.3, -90), group: 'tray' },
  { name: 'tray_4.bottom_left', pose: mm(451.0, -144.7, 96.8, -90), group: 'tray' },
  { name: 'tray_4.bottom_right', pose: mm(451.3, -212.9, 96.8, -90), group: 'tray' },
  { name: 'tray_5.grab', pose: mm(-5.0, 593.8, 129.0, 0), group: 'tray' },
  { name: 'tray_5.bottom_left', pose: mm(-5.5, 692.7, 129.0, 0), group: 'tray' },
  { name: 'tray_5.bottom_right', pose: mm(-54.1, 693.0, 129.0, 0), group: 'tray' },
  { name: 'vial_at_capping_station', pose: capping.grab, group: 'capping' },
  { name: 'vial_cap_at_cap_holder', pose: mm(23.8, -498.0, 152.2, 180), group: 'capping' },
  { name: 'tlh_at_base_station', pose: mm(-43.9, -577.8, 212.2, -180), group: 'tlh' },
  { name: 'tlh_at_capping_station (relative)', pose: { x: capping.grab.x, y: capping.grab.y + 0.0046, z: capping.grab.z + 0.05, rz: capping.grab.rz }, group: 'tlh' },
  { name: 'vial_at_quantos', pose: mm(502.1, -169.0, 191.0, -90), group: 'quantos' },
  { name: 'dosing_head_at_quantos', pose: DOSING_HEADS.quantos.grab, group: 'quantos' },
  ...DOSING_HEADS.rack.map((l, i) => ({ name: `dosing_head_${i + 1}`, pose: l.grab, group: 'heads' as const })),
];

/** The default deck: the location file's bench, with the Quantos. */
export const DECK: DeckDef = buildDeck('quantos');

/** Grab pose of tray slot n (1-based): slots 1..rows run along the file's grab -> bottom_left edge. */
export function slotGrab(n: number, tray: TrayDef = DECK.tray): Pose {
  const i = n - 1;
  const r = i % tray.rows;
  const c = Math.floor(i / tray.rows);
  const fr = tray.rows > 1 ? r / (tray.rows - 1) : 0;
  const fc = tray.cols > 1 ? c / (tray.cols - 1) : 0;
  const g = tray.grab, bl = tray.bottomLeft, br = tray.bottomRight;
  return {
    x: g.x + (bl.x - g.x) * fr + (br.x - bl.x) * fc,
    y: g.y + (bl.y - g.y) * fr + (br.y - bl.y) * fc,
    z: g.z + (bl.z - g.z) * fr + (br.z - bl.z) * fc,
    rz: g.rz,
  };
}

export function slotLocation(n: number, tray: TrayDef = DECK.tray): Location {
  return { grab: slotGrab(n, tray), approach: tray.approach, safe: tray.safe, deckPosition: tray.deckPosition, gripperOpen: tray.gripperOpen };
}

export const TRAY_SLOT_COUNT = DECK.tray.rows * DECK.tray.cols;

export const isTraySlot = (n: unknown): n is number =>
  Number.isInteger(n) && (n as number) >= 1 && (n as number) <= TRAY_SLOT_COUNT;

/** Where the bottom of a vial gripped at `grab` is. */
export const vialSeat = (grab: Pose, vial: VialDef = DECK.vial): Vec3 => [grab.x, grab.y, grab.z - vial.gripHeight];
