// The 3D deck: table, stations, trays, vials, the UR3e (ros-industrial URDF, meshes from a CDN), its
// side-mounted gripper, the liquid handler and the Quantos dosing heads. Everything is placed in the
// robot's `base` frame (z up, metres) inside one group whose matrix turns that into three.js's y-up world.
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { CSS2DObject, CSS2DRenderer } from 'three/examples/jsm/renderers/CSS2DRenderer.js';
import URDFLoader, { type URDFRobot } from 'urdf-loader';
import type { ArmView } from './animator';
import { approachPose, safePose, type Held } from './choreography';
import { DECK, FILE_POINTS, TRAY_SLOT_COUNT, isTraySlot, slotGrab, vialSeat, type DeckDef, type Location, type Pose, type StationDef, type TrayDef, type VialDef } from './deck';
import { UR3, UR3E, fk, jointOrigins, type Joints } from './kinematics';
import type { Mat4, Vec3 } from './mat4';
import type { SimState } from './simulator';

const JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'];
const MESH_CDN = 'https://cdn.jsdelivr.net/gh/ros-industrial/universal_robot@master/ur_description';
const deg = (d: number) => (d * Math.PI) / 180;

export type ViewPreset = 'overview' | 'arm' | 'balance' | 'capping' | 'tray';

/** A cylinder standing on z (three's CylinderGeometry stands on y), with its origin at the bottom. */
function zCylinder(r: number, h: number, material: THREE.Material, rTop = r, segments = 28): THREE.Mesh {
  const g = new THREE.CylinderGeometry(rTop, r, h, segments);
  g.rotateX(Math.PI / 2);
  g.translate(0, 0, h / 2);
  const m = new THREE.Mesh(g, material);
  m.castShadow = true;
  m.receiveShadow = true;
  return m;
}

/** A box standing on its bottom face (origin at the bottom centre). */
function box(w: number, d: number, h: number, material: THREE.Material): THREE.Mesh {
  const g = new THREE.BoxGeometry(w, d, h);
  g.translate(0, 0, h / 2);
  const m = new THREE.Mesh(g, material);
  m.castShadow = true;
  m.receiveShadow = true;
  return m;
}

function label(text: string, className = 'lbl'): CSS2DObject {
  const div = document.createElement('div');
  div.className = className;
  div.textContent = text;
  const obj = new CSS2DObject(div);
  obj.center.set(0.5, 1);
  return obj;
}

function breadboardTexture(dark: boolean): THREE.CanvasTexture {
  const c = document.createElement('canvas');
  c.width = 256;
  c.height = 256;
  const ctx = c.getContext('2d')!;
  ctx.fillStyle = dark ? '#1f2a3d' : '#2b3650';
  ctx.fillRect(0, 0, 256, 256);
  ctx.fillStyle = dark ? '#0f1523' : '#151c2e';
  for (let x = 16; x < 256; x += 32) {
    for (let y = 16; y < 256; y += 32) {
      ctx.beginPath();
      ctx.arc(x, y, 3.2, 0, Math.PI * 2);
      ctx.fill();
    }
  }
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

class Vial {
  group = new THREE.Group();
  liquid: THREE.Mesh;
  solid: THREE.Mesh;
  cap: THREE.Mesh;
  liquidMl = 0;
  solidMg = 0;
  capOn = true;
  capLift = 0;

  constructor(private def: VialDef, glass: THREE.Material, liquidMat: THREE.Material, solidMat: THREE.Material, capMat: THREE.Material) {
    const { radius: r, height: h } = def;
    const g = zCylinder(r, h, glass);
    g.castShadow = false;
    g.receiveShadow = false;
    g.renderOrder = 2;
    this.group.add(g);
    this.solid = zCylinder(r * 0.9, 0.001, solidMat);
    this.liquid = zCylinder(r * 0.9, 0.001, liquidMat);
    this.cap = zCylinder(r * 1.08, 0.012, capMat);
    this.cap.position.z = h - 0.004;
    this.group.add(this.solid, this.liquid, this.cap);
    this.render();
  }

  setContents(solidMg: number, liquidMl: number) {
    this.solidMg = solidMg;
    this.liquidMl = liquidMl;
    this.render();
  }

  setCap(on: boolean, lift = 0) {
    this.capOn = on;
    this.capLift = lift;
    this.render();
  }

  render() {
    const h = this.def.height;
    const sH = this.solidMg > 0 ? Math.min(0.3, 0.05 + this.solidMg / 800) * h : 0;
    const lH = this.liquidMl > 0 ? Math.min(0.8 - sH / h, this.liquidMl * 0.045) * h : 0;
    this.solid.visible = sH > 0;
    this.solid.scale.z = Math.max(sH, 0.0005) / 0.001;
    this.liquid.visible = lH > 0;
    this.liquid.scale.z = Math.max(lH, 0.0005) / 0.001;
    this.liquid.position.z = sH;
    this.cap.visible = this.capOn || this.capLift > 0;
    this.cap.position.z = h - 0.004 + this.capLift;
  }
}

interface StationView {
  def: StationDef;
  group: THREE.Group;
  body: THREE.Mesh | null;
  valueLabel?: HTMLDivElement;
  extras: Record<string, THREE.Object3D>;
  door?: { mesh: THREE.Mesh; closedY: number; travel: number };
}

export interface SceneApi extends ArmView {
  debug(): Record<string, unknown>;
  applyState(state: SimState): void;
  tick(dt: number): void;
  setSafePoints(visible: boolean): void;
  setTrail(visible: boolean): void;
  setLabels(visible: boolean): void;
  setLocationsOnly(on: boolean): void;
  clearTrail(): void;
  resize(): void;
  view(preset: ViewPreset): void;
  armReady: Promise<void>;
}

export function createScene(container: HTMLElement, deck: DeckDef = DECK): SceneApi {
  const params = deck.arm === 'UR3e' ? UR3E : UR3;
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  container.appendChild(renderer.domElement);

  const labelRenderer = new CSS2DRenderer();
  labelRenderer.domElement.className = 'labels';
  container.appendChild(labelRenderer.domElement);

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(42, 1, 0.02, 20);
  // the canvas takes the mouse (the label layer above it lets events through), so drag orbits, wheel zooms, right-drag pans
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.maxPolarAngle = Math.PI / 2 - 0.02;
  controls.minDistance = 0.15;
  controls.maxDistance = 4;

  // base frame (x, y, z up) -> three (x right, y up, z toward the viewer): base x -> three z, base y -> three x
  const deckGroup = new THREE.Group();
  deckGroup.matrixAutoUpdate = false;
  deckGroup.matrix.makeBasis(new THREE.Vector3(0, 0, 1), new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 1, 0));
  scene.add(deckGroup);
  // everything that stands on the deck (hidden by the "locations only" view); the table, the arm,
  // the waypoint dots and the file-point markers live directly in deckGroup
  const objects = new THREE.Group();
  deckGroup.add(objects);
  const toWorld = (p: Vec3) => new THREE.Vector3(p[0], p[1], p[2]).applyMatrix4(deckGroup.matrix);

  // lights
  scene.add(new THREE.HemisphereLight(0xffffff, 0x8899aa, 0.9));
  const sun = new THREE.DirectionalLight(0xffffff, 2.2);
  sun.position.set(0.8, 1.6, 1.2);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  sun.shadow.camera.left = -1.2;
  sun.shadow.camera.right = 1.2;
  sun.shadow.camera.top = 1.2;
  sun.shadow.camera.bottom = -1.2;
  sun.shadow.camera.near = 0.2;
  sun.shadow.camera.far = 5;
  sun.shadow.bias = -0.0004;
  sun.shadow.normalBias = 0.015;
  scene.add(sun);
  const fill = new THREE.DirectionalLight(0xffffff, 0.6);
  fill.position.set(-1, 0.8, -0.6);
  scene.add(fill);

  // materials
  const glassMat = new THREE.MeshPhysicalMaterial({ color: 0xdfe8ff, transparent: true, opacity: 0.32, roughness: 0.15, metalness: 0, depthWrite: false });
  const liquidMat = new THREE.MeshStandardMaterial({ color: 0x7c3aed, transparent: true, opacity: 0.75, roughness: 0.3 });
  const solidMat = new THREE.MeshStandardMaterial({ color: 0xf59e0b, roughness: 0.9 });
  const capMat = new THREE.MeshStandardMaterial({ color: 0x1f2937, roughness: 0.6 });
  const whiteMat = new THREE.MeshStandardMaterial({ color: 0xf3f4f6, roughness: 0.55 });
  const steelMat = new THREE.MeshStandardMaterial({ color: 0xb8bec8, metalness: 0.8, roughness: 0.35 });
  const darkMat = new THREE.MeshStandardMaterial({ color: 0x374151, roughness: 0.7 });
  const rackMat = new THREE.MeshStandardMaterial({ color: 0xd8d2c4, roughness: 0.7 });
  const holeMat = new THREE.MeshStandardMaterial({ color: 0x0f172a, roughness: 1 });
  const accentMat = new THREE.MeshStandardMaterial({ color: 0x38bdf8, emissive: 0x38bdf8, emissiveIntensity: 0.4, transparent: true, opacity: 0.55 });
  const safeMat = new THREE.MeshStandardMaterial({ color: 0xf97316, emissive: 0xf97316, emissiveIntensity: 0.3, transparent: true, opacity: 0.45 });

  // ── table and mount ─────────────────────────────────────────────────────────────────────────
  const tableTex = breadboardTexture(document.documentElement.classList.contains('dark'));
  const t = deck.table;
  const tw = t.x1 - t.x0, td = t.y1 - t.y0;
  tableTex.repeat.set(tw / 0.1, td / 0.1);
  const table = box(tw, td, t.thickness, new THREE.MeshStandardMaterial({ map: tableTex, roughness: 0.85 }));
  table.position.set((t.x0 + t.x1) / 2, (t.y0 + t.y1) / 2, t.top - t.thickness);
  table.castShadow = false;
  deckGroup.add(table);
  const mount = box(deck.mount.w, deck.mount.d, deck.mount.h, darkMat);
  mount.position.set(0, 0, -deck.mount.h);
  deckGroup.add(mount);

  const labels: CSS2DObject[] = [];
  const safePoints: THREE.Object3D[] = [];
  const pointGeom = new THREE.SphereGeometry(0.006, 12, 12);
  const addLabel = (text: string, at: Vec3, cls = 'lbl', parent: THREE.Object3D = objects) => {
    const l = label(text, cls);
    l.position.set(...at);
    parent.add(l);
    labels.push(l);
    return l;
  };
  /** The approach and safe points of a location: the blue and orange dots of the "safe-height points" toggle. */
  const addWaypoints = (l: Location) => {
    const a = approachPose(l), s = safePose(l);
    const pa = new THREE.Mesh(pointGeom, accentMat);
    pa.position.set(a.x, a.y, a.z);
    const ps = new THREE.Mesh(pointGeom, safeMat);
    ps.position.set(s.x, s.y, s.z);
    const line = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(l.grab.x, l.grab.y, l.grab.z), new THREE.Vector3(a.x, a.y, a.z), new THREE.Vector3(s.x, s.y, s.z)]),
      new THREE.LineDashedMaterial({ color: 0x94a3b8, dashSize: 0.006, gapSize: 0.004, transparent: true, opacity: 0.7 }),
    );
    line.computeLineDistances();
    deckGroup.add(pa, ps, line);
    safePoints.push(pa, ps, line);
  };

  // ── trays ───────────────────────────────────────────────────────────────────────────────────
  const vials: Record<number, Vial> = {};
  function buildTray(tray: TrayDef, withVials: boolean, main: boolean) {
    const g = tray.grab, bl = tray.bottomLeft, br = tray.bottomRight;
    const seatZ = g.z - tray.vial.gripHeight;
    const rackTop = seatZ + 0.016;
    const cx = (g.x + br.x) / 2, cy = (g.y + br.y) / 2;
    const w = Math.abs(bl.x - g.x) + 0.045, d = Math.abs(br.y - bl.y) + 0.045;
    const rack = box(w, d, rackTop - t.top, rackMat);
    rack.position.set(cx, cy, t.top);
    objects.add(rack);
    addLabel(tray.label, [cx, cy, rackTop + tray.vial.height + 0.04], main ? 'lbl' : 'lbl lbl-dim');
    const count = tray.rows * tray.cols;
    for (let n = 1; n <= count; n++) {
      const p = slotGrab(n, tray);
      const hole = zCylinder(tray.vial.radius * 1.2, 0.0178, holeMat);
      hole.position.set(p.x, p.y, seatZ - 0.001);
      hole.castShadow = false;
      objects.add(hole);
      if (withVials) {
        const v = new Vial(tray.vial, glassMat, liquidMat, solidMat, capMat);
        v.group.position.set(...vialSeat(p, tray.vial));
        objects.add(v.group);
        if (main) {
          vials[n] = v;
          const num = label(String(n), 'lbl lbl-slot');
          num.position.set(p.x, p.y, rackTop + 0.001);
          num.center.set(0.5, 0.5);
          objects.add(num);
          labels.push(num);
        }
      }
    }
    if (main) addWaypoints({ grab: g, approach: tray.approach, safe: tray.safe, deckPosition: tray.deckPosition, gripperOpen: tray.gripperOpen });
  }
  buildTray(deck.tray, true, true);
  for (const tr of deck.otherTrays) buildTray(tr, tr.vial.height < 0.05, false);

  // ── stations ────────────────────────────────────────────────────────────────────────────────
  const stations: Record<string, StationView> = {};
  for (const def of Object.values(deck.stations)) {
    if (def.virtual) continue;
    const group = new THREE.Group();
    const seat = vialSeat(def.location.grab, deck.vial);
    const extras: Record<string, THREE.Object3D> = {};
    let body: THREE.Mesh | null = null;
    let door: StationView['door'];
    const fp = def.footprint;
    const center: Vec3 = def.center ? [def.center[0], def.center[1], t.top] : [seat[0], seat[1], t.top];
    const mat = new THREE.MeshStandardMaterial({ color: new THREE.Color(def.color), roughness: 0.6 });

    if (def.draw === 'box' || def.draw === 'balance') {
      body = box(fp.w, fp.d, fp.h, mat);
      body.position.set(...center);
      group.add(body);
      const boxTop = t.top + fp.h;
      if (def.draw === 'balance') {
        const pan = zCylinder(0.055, 0.004, steelMat);
        pan.position.set(seat[0], seat[1], boxTop);
        const holder = zCylinder(0.02, seat[2] - boxTop - 0.004, steelMat);
        holder.position.set(seat[0], seat[1], boxTop + 0.004);
        const display = box(0.04, 0.06, 0.012, darkMat);
        display.position.set(center[0] + fp.w / 2 - 0.03, center[1] + fp.d / 2 - 0.04, boxTop);
        group.add(pan, holder, display);
      } else if (seat[2] > boxTop + 1e-4) {
        const ped = zCylinder(0.022, seat[2] - boxTop, steelMat);
        ped.position.set(seat[0], seat[1], boxTop);
        group.add(ped);
      }
    } else if (def.draw === 'capper') {
      // a box up to the vial seat with two jaws beside the vial: they close on it and spin it while
      // the arm holds the cap still. Nothing above the vial.
      body = box(fp.w, fp.d, seat[2] - t.top, mat);
      body.position.set(...center);
      const jawL = box(0.022, 0.05, 0.03, darkMat);
      const jawR = box(0.022, 0.05, 0.03, darkMat);
      jawL.position.set(seat[0] - 0.035, seat[1], seat[2] + 0.008);
      jawR.position.set(seat[0] + 0.035, seat[1], seat[2] + 0.008);
      const rail = box(0.1, 0.02, 0.006, steelMat);
      rail.position.set(seat[0], seat[1] + 0.03, seat[2]);
      group.add(body, jawL, jawR, rail);
      extras.jawL = jawL;
      extras.jawR = jawR;
    } else if (def.draw === 'quantos') {
      // An XPR-style Quantos: the weighing chamber is the front third (toward +y, where the robot's
      // side door is), the balance body the back two thirds. The dosing module sits on top of the
      // chamber with the head hook under its front edge (the file's dosing_head_at_quantos point);
      // the carrier lowers the head onto the vial on the pan below.
      const q = deck.dosingHeads!.quantos.grab;
      const front = center[1] + fp.d / 2;          // +y face, the chamber front
      const chamberD = 0.16;
      const chamberH = 0.30;
      const glassy = new THREE.MeshPhysicalMaterial({ color: 0xe8eef7, roughness: 0.1, transparent: true, opacity: 0.3, depthWrite: false });
      // balance body at the back, lower, with the display on its top
      const bodyBox = box(fp.w, fp.d - chamberD, 0.22, mat);
      bodyBox.position.set(center[0], front - chamberD - (fp.d - chamberD) / 2, t.top);
      const display = box(0.08, 0.05, 0.01, darkMat);
      display.position.set(center[0], front - chamberD - 0.05, t.top + 0.22);
      // chamber floor (the weighing pan level), glass walls on three sides, the robot's side is the door
      const floor = box(fp.w, chamberD, seat[2] - 0.004 - t.top, mat);
      floor.position.set(center[0], front - chamberD / 2, t.top);
      const pan = zCylinder(0.035, 0.004, steelMat);
      pan.position.set(seat[0], seat[1], seat[2] - 0.004);
      const glassFront = box(fp.w, 0.01, chamberH, glassy);
      glassFront.position.set(center[0], front - 0.005, t.top);
      const glassBack = box(fp.w, 0.01, chamberH, glassy);
      glassBack.position.set(center[0], front - chamberD + 0.005, t.top);
      const glassFar = box(0.01, chamberD, chamberH, glassy);
      glassFar.position.set(center[0] + fp.w / 2 - 0.005, front - chamberD / 2, t.top);
      const frame1 = box(0.012, 0.012, chamberH, steelMat);
      frame1.position.set(center[0] + fp.w / 2 - 0.006, front - 0.006, t.top);
      const frame2 = box(0.012, 0.012, chamberH, steelMat);
      frame2.position.set(center[0] - fp.w / 2 + 0.006, front - 0.006, t.top);
      // the dosing module on top of the chamber, and the carrier with the hook the gun hangs from
      const moduleBox = box(fp.w, chamberD + 0.02, 0.09, mat);
      moduleBox.position.set(center[0], front - chamberD / 2 + 0.01, t.top + chamberH);
      const carrier = new THREE.Group();
      const carrierArm = box(0.05, 0.03, 0.03, darkMat);
      carrierArm.position.set(0, 0, 0);
      const hook = box(0.03, 0.012, 0.012, steelMat);
      hook.position.set(0, -0.01, -0.012);
      carrier.add(carrierArm, hook);
      carrier.position.set(q.x, q.y + 0.02, q.z + 0.02);
      // the side door on the robot's side, sliding to the back (-y)
      const doorMesh = box(0.01, chamberD - 0.02, chamberH - 0.02, new THREE.MeshPhysicalMaterial({ color: 0xcfe3ff, roughness: 0.05, transparent: true, opacity: 0.35, depthWrite: false }));
      doorMesh.position.set(center[0] - fp.w / 2 + 0.005, front - chamberD / 2, t.top + 0.01);
      door = { mesh: doorMesh, closedY: front - chamberD / 2, travel: -(chamberD - 0.02) };
      const stream = zCylinder(0.003, 0.05, solidMat);
      stream.position.set(seat[0], seat[1], seat[2] + deck.vial.height);
      stream.visible = false;
      group.add(bodyBox, display, floor, pan, glassFront, glassBack, glassFar, frame1, frame2, moduleBox, carrier, doorMesh, stream);
      body = floor;
      extras.stream = stream;
      extras.carrier = carrier;
    }

    if (def.draw !== 'none') {
      const spot = zCylinder(deck.vial.radius * 1.25, 0.002, holeMat);
      spot.position.set(seat[0], seat[1], seat[2] - 0.002);
      group.add(spot);
      addWaypoints(def.location);
    }

    const tag = def.estimated ? ' (estimated)' : '';
    const labelZ = def.draw === 'quantos' ? t.top + 0.42 : seat[2] + deck.vial.height + 0.06;
    const lz = def.sharesSeatWith ? labelZ - 0.035 : labelZ;
    addLabel(def.label + tag, [seat[0], seat[1], lz], def.sharesSeatWith ? 'lbl lbl-dim' : 'lbl', group);

    let valueLabel: HTMLDivElement | undefined;
    if (def.id === 'weigh_balance') {
      const at: Vec3 = def.draw === 'quantos'
        ? [center[0], center[1] - 0.12, t.top + 0.24]
        : [center[0] + fp.w / 2 - 0.03, center[1] + fp.d / 2 - 0.04, t.top + fp.h + 0.015];
      const v = addLabel('0.0 mg', at, 'lbl lbl-value', group);
      valueLabel = v.element as HTMLDivElement;
    }
    objects.add(group);
    stations[def.id] = { def, group, body, valueLabel, extras, door };
  }

  // ── cap holder, dosing-head rack, liquid handler wash station ──────────────────────────────
  {
    const c = deck.capHolder.grab;
    const post = zCylinder(0.01, c.z - 0.02 - t.top, steelMat);
    post.position.set(c.x, c.y, t.top);
    const cup = zCylinder(deck.vial.radius * 1.4, 0.02, whiteMat);
    cup.position.set(c.x, c.y, c.z - 0.02);
    objects.add(post, cup);
    addLabel('Cap holder', [c.x, c.y, c.z + 0.03], 'lbl lbl-dim');
  }

  let cameraLens: THREE.Mesh | null = null;
  const heads: THREE.Group[] = [];
  const headMat = new THREE.MeshPhysicalMaterial({ color: 0xf1f5f9, roughness: 0.6, transparent: true, opacity: 0.8 });
  const makeHead = () => {
    // a powder cartridge: a tube with a white cap on top and the dosing valve at the bottom
    const g = new THREE.Group();
    const body = zCylinder(0.015, 0.06, headMat);
    body.position.z = -0.03;
    const cap = zCylinder(0.018, 0.012, whiteMat);
    cap.position.z = 0.03;
    const valve = zCylinder(0.008, 0.015, steelMat);
    valve.position.z = -0.045;
    const powder = zCylinder(0.013, 0.03, solidMat);
    powder.position.z = -0.028;
    g.add(body, cap, valve, powder);
    return g;
  };
  if (deck.dosingHeads) {
    const rack = deck.dosingHeads.rack;
    const xs = rack.map((l) => l.grab.x), ys = rack.map((l) => l.grab.y), zs = rack.map((l) => l.grab.z);
    const panel = box(0.02, Math.max(...ys) - Math.min(...ys) + 0.09, Math.max(...zs) + 0.05 - t.top, whiteMat);
    panel.position.set(Math.max(...xs) + 0.03, (Math.max(...ys) + Math.min(...ys)) / 2, t.top);
    objects.add(panel);
    rack.forEach((l, i) => {
      const shelf = box(0.05, 0.05, 0.006, steelMat);
      shelf.position.set(l.grab.x, l.grab.y, l.grab.z - 0.051);
      objects.add(shelf);
      const h = makeHead();
      h.position.set(l.grab.x, l.grab.y, l.grab.z);
      h.rotation.z = deg(l.grab.rz);
      objects.add(h);
      heads.push(h);
      if (i === 0) addWaypoints(l);
    });
    addLabel('Dosing heads', [Math.max(...xs), (Math.max(...ys) + Math.min(...ys)) / 2, Math.max(...zs) + 0.07], 'lbl lbl-dim');
    addWaypoints(deck.dosingHeads.quantos);
  }

  const tlh = new THREE.Group();
  {
    const b = deck.tlh.base.grab;
    const tip = deck.tlh.needleTip;
    // the gripped block, a collar under it and the needle: its tip `needleTip` below the TCP
    const body = box(0.024, 0.02, 0.05, whiteMat);
    body.position.z = -0.015;
    const collar = zCylinder(0.005, 0.01, steelMat);
    collar.position.z = -0.025;
    const needle = zCylinder(0.0012, tip - 0.025, steelMat);
    needle.position.z = -tip;
    const line = new THREE.Mesh(new THREE.TorusGeometry(0.018, 0.0025, 8, 24, Math.PI), new THREE.MeshStandardMaterial({ color: 0xd946ef }));
    line.position.z = 0.035;
    line.rotation.y = Math.PI / 2;
    tlh.add(body, collar, needle, line);
    tlh.position.set(b.x, b.y, b.z);
    tlh.rotation.z = deg(b.rz);
    objects.add(tlh);
    // the wash station behind it (away from the robot, at the far end): a block with the wash cup the
    // needle parks in, a bracket holding the body from behind, and tubing running out of the bottom
    const behind = b.y - 0.045; // the TLH faces the robot (+y side)
    const station = box(0.07, 0.05, b.z - tip - t.top + 0.01, whiteMat);
    station.position.set(b.x, behind, t.top);
    const bracket = box(0.03, 0.05, 0.008, steelMat);
    bracket.position.set(b.x, b.y - 0.025, b.z - 0.014);
    const cup = zCylinder(0.011, 0.03, whiteMat);
    cup.position.set(b.x, b.y, b.z - tip - 0.005);
    const stem = zCylinder(0.006, b.z - tip - 0.005 - t.top, steelMat);
    stem.position.set(b.x, b.y, t.top);
    const tubing = new THREE.Mesh(new THREE.TorusGeometry(0.03, 0.003, 8, 24, Math.PI), new THREE.MeshStandardMaterial({ color: 0xe5e7eb }));
    tubing.position.set(b.x - 0.02, behind - 0.03, t.top + 0.004);
    tubing.rotation.x = Math.PI / 2;
    tubing.rotation.z = Math.PI / 2;
    const tubing2 = new THREE.Mesh(new THREE.TorusGeometry(0.03, 0.003, 8, 24, Math.PI), new THREE.MeshStandardMaterial({ color: 0xe5e7eb }));
    tubing2.position.set(b.x + 0.02, behind - 0.03, t.top + 0.004);
    tubing2.rotation.x = Math.PI / 2;
    tubing2.rotation.z = Math.PI / 2;
    objects.add(station, bracket, cup, stem, tubing, tubing2);
    addLabel(deck.tlh.label, [b.x, b.y, b.z + 0.07]);
    addWaypoints(deck.tlh.base);
  }

  // the loose cap: on a vial, in the gripper, or in the cap holder (the arm carries it between them)
  const looseCap = zCylinder(deck.vial.radius * 1.08, 0.012, capMat);
  looseCap.visible = false;
  objects.add(looseCap);

  // the webcam the arm shows the vial to
  if (deck.camera) {
    const c = deck.camera.position;
    const post = zCylinder(0.008, c[2] - 0.02 - t.top, steelMat);
    post.position.set(c[0], c[1], t.top);
    const foot = zCylinder(0.03, 0.008, darkMat);
    foot.position.set(c[0], c[1], t.top);
    const cam = new THREE.Group();
    const bodyMesh = new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.09, 0.03), darkMat);
    const lens = new THREE.Mesh(new THREE.CylinderGeometry(0.01, 0.01, 0.012, 20), new THREE.MeshStandardMaterial({ color: 0x0ea5e9, emissive: 0x0ea5e9, emissiveIntensity: 0.2 }));
    lens.rotation.z = Math.PI / 2;
    lens.position.x = 0.02;
    cam.add(bodyMesh, lens);
    cam.position.set(c[0], c[1], c[2]);
    const hold = deck.stations.analysis_station?.location.grab;
    if (hold) cam.rotation.z = Math.atan2(hold.y - c[1], hold.x - c[0]); // the lens faces where the vial is shown
    objects.add(post, foot, cam);
    addLabel(deck.camera.label + ' (estimated)', [c[0], c[1], c[2] + 0.04], 'lbl lbl-dim');
    cameraLens = lens;
  }

  // ── scenery ─────────────────────────────────────────────────────────────────────────────────
  for (const s of deck.scenery) {
    const fp = s.footprint;
    const group = new THREE.Group();
    const mat = new THREE.MeshStandardMaterial({ color: new THREE.Color(s.color), roughness: 0.7, metalness: s.kind === 'cylinder' ? 0.6 : 0 });
    const b = s.kind === 'cylinder' ? zCylinder(fp.w / 2, fp.h, mat) : box(fp.w, fp.d, fp.h, mat);
    b.position.set(...s.center);
    group.add(b);
    if (s.kind === 'bottles') {
      const bottleMat = new THREE.MeshPhysicalMaterial({ color: 0xd9e4ff, transparent: true, opacity: 0.45, roughness: 0.1, depthWrite: false });
      const purple = new THREE.MeshStandardMaterial({ color: 0x9333ea, roughness: 0.5 });
      const fills = [0x1e3a8a, 0x15803d, 0xca8a04, 0x6b21a8, 0x0f766e, 0x7f1d1d, 0x1d4ed8, 0x374151];
      let i = 0;
      for (let r = 0; r < 2; r++) {
        for (let c = 0; c < 4; c++) {
          const x = s.center[0] - fp.w / 2 + 0.045 + c * 0.075;
          const y = s.center[1] - fp.d / 2 + 0.055 + r * 0.11;
          const z = s.center[2] + 0.006;
          const bottle = zCylinder(0.032, 0.15, bottleMat);
          bottle.position.set(x, y, z);
          const content = zCylinder(0.029, 0.04 + ((i * 37) % 80) / 1000, new THREE.MeshStandardMaterial({ color: fills[i++ % fills.length], transparent: true, opacity: 0.8 }));
          content.position.set(x, y, z + 0.002);
          const cap = zCylinder(0.02, 0.022, purple);
          cap.position.set(x, y, z + 0.15);
          group.add(bottle, content, cap);
        }
      }
    }
    if (s.id === 'vici_valve') {
      const knob = zCylinder(0.018, 0.012, steelMat);
      knob.position.set(s.center[0], s.center[1], s.center[2] + fp.h);
      group.add(knob);
    }
    addLabel(s.label + (s.estimated ? ' (estimated)' : ''), [s.center[0], s.center[1], s.center[2] + fp.h + (s.kind === 'bottles' ? 0.2 : 0.03)], 'lbl lbl-dim', group);
    objects.add(group);
  }

  // ── the arm ─────────────────────────────────────────────────────────────────────────────────
  const robotRoot = new THREE.Group();
  robotRoot.rotation.z = Math.PI; // base_link is the pendant's `base` frame turned by 180 degrees
  deckGroup.add(robotRoot);

  // a stick figure from the kinematics, shown until the meshes arrive (and if they never do)
  const skeleton = new THREE.Group();
  const skelMat = new THREE.MeshStandardMaterial({ color: 0x94a3b8, roughness: 0.5 });
  const skelJoints: THREE.Mesh[] = [];
  const skelLinks: THREE.Mesh[] = [];
  for (let i = 0; i < 7; i++) {
    const j = new THREE.Mesh(new THREE.SphereGeometry(0.022, 16, 16), skelMat);
    skeleton.add(j);
    skelJoints.push(j);
    if (i < 6) {
      const l = new THREE.Mesh(new THREE.CylinderGeometry(0.014, 0.014, 1, 12), skelMat);
      skeleton.add(l);
      skelLinks.push(l);
    }
  }
  deckGroup.add(skeleton);
  const up = new THREE.Vector3(0, 1, 0);
  const updateSkeleton = (q: Joints) => {
    const pts = jointOrigins(q, params);
    pts.forEach((p, i) => skelJoints[i].position.set(p[0], p[1], p[2]));
    for (let i = 0; i < 6; i++) {
      const a = new THREE.Vector3(...pts[i]);
      const b = new THREE.Vector3(...pts[i + 1]);
      const l = skelLinks[i];
      const len = a.distanceTo(b);
      l.visible = len > 1e-4;
      l.scale.set(1, Math.max(len, 1e-4), 1);
      l.position.copy(a).lerp(b, 0.5);
      l.quaternion.setFromUnitVectors(up, b.clone().sub(a).normalize());
    }
  };

  // the gripper, as on the robot: an L bracket from the flange to a two-finger gripper 200 mm to the
  // side (deck.tcp); the fingers are plates that close on the vial from both sides
  const toolGroup = new THREE.Group();
  {
    const [tx, ty, tz] = deck.tcp.xyz;
    const alu = new THREE.MeshStandardMaterial({ color: 0xc7ccd4, metalness: 0.75, roughness: 0.3 });
    const adapter = zCylinder(0.031, 0.01, darkMat);
    // a flat aluminium plate from the flange out to the gripper, with lightening holes suggested by a second thin plate
    const plate = new THREE.Mesh(new THREE.BoxGeometry(0.05, Math.abs(ty) + 0.04, 0.008), alu);
    plate.position.set(tx, ty / 2, 0.014);
    plate.castShadow = true;
    const rib = new THREE.Mesh(new THREE.BoxGeometry(0.012, Math.abs(ty) - 0.02, 0.014), alu);
    rib.position.set(tx, ty / 2, 0.025);
    rib.castShadow = true;
    // the Hand-E style body hanging under the end of the plate, its fingers around the TCP
    const gripBody = zCylinder(0.027, 0.045, new THREE.MeshStandardMaterial({ color: 0x1f2937, roughness: 0.4 }));
    gripBody.position.set(tx, ty, 0.018);
    const collar = zCylinder(0.03, 0.006, new THREE.MeshStandardMaterial({ color: 0x2563eb, roughness: 0.4 }));
    collar.position.set(tx, ty, 0.018);
    const carriage = new THREE.Mesh(new THREE.BoxGeometry(0.04, 0.07, 0.014), alu);
    carriage.position.set(tx, ty, tz - 0.03 + 0.007);
    carriage.castShadow = true;
    toolGroup.add(adapter, plate, rib, gripBody, collar, carriage);
  }
  const fingerMat = new THREE.MeshStandardMaterial({ color: 0xb8bec8, metalness: 0.7, roughness: 0.35 });
  const fingerL = new THREE.Mesh(new THREE.BoxGeometry(0.022, 0.004, 0.044), fingerMat);
  const fingerR = new THREE.Mesh(new THREE.BoxGeometry(0.022, 0.004, 0.044), fingerMat);
  fingerL.castShadow = true;
  fingerR.castShadow = true;
  toolGroup.add(fingerL, fingerR);
  const tcp = new THREE.Group();
  tcp.position.set(...deck.tcp.xyz);
  tcp.rotation.x = deck.tcp.rx; // a held object stands up along the TCP z
  toolGroup.add(tcp);
  tcp.add(new THREE.Mesh(new THREE.SphereGeometry(0.004, 10, 10), new THREE.MeshBasicMaterial({ color: 0x06b6d4 })));
  const toolCarrier = new THREE.Group(); // holds the tool at the flange's fk pose until tool0 exists
  toolCarrier.add(toolGroup);
  deckGroup.add(toolCarrier);
  const setFingers = (open: number) => {
    const [tx, ty, tz] = deck.tcp.xyz;
    const half = deck.vial.radius + 0.002 + open * 0.03;
    fingerL.position.set(tx, ty + half, tz - 0.002);
    fingerR.position.set(tx, ty - half, tz - 0.002);
  };
  setFingers(1);

  let robot: URDFRobot | null = null;
  let lastJoints: Joints = [...deck.homes.positive_x] as Joints;
  const matFromMat4 = (m: Mat4) => new THREE.Matrix4().set(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8], m[9], m[10], m[11], 0, 0, 0, 1);

  function setJoints(q: Joints) {
    lastJoints = q;
    if (robot) {
      JOINT_NAMES.forEach((name, i) => robot!.joints[name]?.setJointValue(q[i]));
    } else {
      updateSkeleton(q);
      toolCarrier.matrixAutoUpdate = false;
      toolCarrier.matrix.copy(matFromMat4(fk(q, params)));
    }
  }

  const armReady = new Promise<void>((resolve) => {
    const loader = new URDFLoader();
    loader.packages = { ur_description: MESH_CDN };
    loader.load(
      new URL(deck.arm === 'UR3e' ? 'ur3e.urdf' : 'ur3.urdf', location.href).toString(),
      (r) => {
        robot = r;
        r.traverse((o) => {
          if ((o as THREE.Mesh).isMesh) {
            o.castShadow = true;
            o.receiveShadow = true;
          }
        });
        robotRoot.add(r);
        const tool0 = r.links['tool0'];
        if (tool0) {
          toolCarrier.remove(toolGroup);
          tool0.add(toolGroup);
        }
        skeleton.visible = false;
        setJoints(lastJoints);
        resolve();
      },
      undefined,
      (err) => {
        console.warn('[beavers-bench] arm model failed to load, keeping the stick figure', err);
        resolve();
      },
    );
  });

  // ── the file's engagement points, for the "locations only" view ────────────────────────────
  const markers = new THREE.Group();
  markers.visible = false;
  deckGroup.add(markers);
  const markerLabels: CSS2DObject[] = [];
  {
    const colours: Record<string, number> = { home: 0x64748b, tray: 0x2563eb, capping: 0xdb2777, quantos: 0xd97706, heads: 0xca8a04, tlh: 0x7c3aed };
    const mm3 = (p: Pose) => `${(p.x * 1000).toFixed(1)}, ${(p.y * 1000).toFixed(1)}, ${(p.z * 1000).toFixed(1)} · rz ${p.rz}`;
    for (const fp of FILE_POINTS) {
      const col = colours[fp.group] ?? 0x64748b;
      const dot = new THREE.Mesh(new THREE.SphereGeometry(fp.group === 'home' ? 0.012 : 0.008, 14, 14), new THREE.MeshStandardMaterial({ color: col, emissive: col, emissiveIntensity: 0.35 }));
      dot.position.set(fp.pose.x, fp.pose.y, fp.pose.z);
      // the TCP's x axis at that pose (its yaw rz), and a drop line to the table
      const yaw = deg(fp.pose.rz);
      const axis = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(fp.pose.x, fp.pose.y, fp.pose.z), new THREE.Vector3(fp.pose.x + 0.04 * Math.cos(yaw), fp.pose.y + 0.04 * Math.sin(yaw), fp.pose.z)]),
        new THREE.LineBasicMaterial({ color: 0xef4444 }),
      );
      const drop = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(fp.pose.x, fp.pose.y, fp.pose.z), new THREE.Vector3(fp.pose.x, fp.pose.y, t.top)]),
        new THREE.LineDashedMaterial({ color: col, dashSize: 0.008, gapSize: 0.006, transparent: true, opacity: 0.6 }),
      );
      drop.computeLineDistances();
      const div = document.createElement('div');
      div.className = 'lbl lbl-point';
      div.innerHTML = `<b>${fp.name}</b><br>${mm3(fp.pose)}`;
      const lbl = new CSS2DObject(div);
      lbl.center.set(0, 1);
      lbl.position.set(fp.pose.x + 0.01, fp.pose.y, fp.pose.z + 0.012);
      markers.add(dot, axis, drop, lbl);
      markerLabels.push(lbl);
    }
    // the tray slots the simulator uses, as small dots between the corner points
    for (let n = 1; n <= TRAY_SLOT_COUNT; n++) {
      const p = slotGrab(n, deck.tray);
      const dot = new THREE.Mesh(new THREE.SphereGeometry(0.004, 10, 10), new THREE.MeshStandardMaterial({ color: 0x2563eb }));
      dot.position.set(p.x, p.y, p.z);
      markers.add(dot);
    }
    // the base frame's axes at the robot's origin
    const axes = new THREE.AxesHelper(0.15);
    markers.add(axes);
  }

  // ── trail ───────────────────────────────────────────────────────────────────────────────────
  const TRAIL_MAX = 400;
  const trailPositions = new Float32Array(TRAIL_MAX * 3);
  const trailGeom = new THREE.BufferGeometry();
  trailGeom.setAttribute('position', new THREE.BufferAttribute(trailPositions, 3));
  trailGeom.setDrawRange(0, 0);
  const trail = new THREE.Line(trailGeom, new THREE.LineBasicMaterial({ color: 0xf97316, transparent: true, opacity: 0.8 }));
  trail.frustumCulled = false;
  deckGroup.add(trail);
  let trailCount = 0;

  // ── state ───────────────────────────────────────────────────────────────────────────────────
  let held: { what: Held; obj: THREE.Object3D; vial?: Vial } | null = null;
  let dockedHead: THREE.Object3D | null = null; // the gun hanging on the Quantos hook
  const vialAt: Record<string, Vial | undefined> = {};
  const flashes = new Map<THREE.Mesh, number>();
  const doorTargets = new Map<string, number>(); // station -> 0 closed .. 1 open

  const seatFor = (at: string, slot?: number): Vec3 => {
    if (at === 'tray' && slot && isTraySlot(slot)) return vialSeat(slotGrab(slot, deck.tray), deck.vial);
    const st = deck.stations[at];
    return st ? vialSeat(st.location.grab, deck.vial) : [0, 0, 0];
  };

  function noteVialAt(v: Vial, at: string) {
    for (const k of Object.keys(vialAt)) if (vialAt[k] === v) delete vialAt[k];
    if (at === 'tray') return;
    vialAt[at] = v;
    // one instrument, two roles: the vial is at both
    const shared = Object.values(deck.stations).find((s) => s.sharesSeatWith === at)?.id;
    if (shared) vialAt[shared] = v;
    const owner = deck.stations[at]?.sharesSeatWith;
    if (owner) vialAt[owner] = v;
  }

  function putVial(v: Vial, at: string, slot?: number) {
    if (v.group.parent !== objects) {
      v.group.removeFromParent();
      objects.add(v.group);
    }
    v.group.rotation.set(0, 0, 0);
    v.group.position.set(...seatFor(at, slot));
    noteVialAt(v, at);
  }

  function park(obj: THREE.Object3D, p: Pose, tiltX = 0) {
    obj.removeFromParent();
    objects.add(obj);
    obj.position.set(p.x, p.y, p.z);
    obj.rotation.set(tiltX, 0, deg(p.rz));
  }

  function holdObject(obj: THREE.Object3D, dz: number) {
    obj.removeFromParent();
    obj.position.set(0, 0, dz);
    obj.rotation.set(0, 0, 0);
    tcp.add(obj);
  }

  function holdVial(v: Vial) {
    holdObject(v.group, -deck.vial.gripHeight);
    held = { what: 'vial', obj: v.group, vial: v };
  }

  function tcpWorld(): THREE.Vector3 {
    const v = new THREE.Vector3();
    tcp.getWorldPosition(v);
    return v;
  }

  function nearestTo(candidates: THREE.Object3D[], radius: number): THREE.Object3D | null {
    const here = tcpWorld();
    let best: THREE.Object3D | null = null;
    let bestD = radius;
    for (const o of candidates) {
      if (o.parent !== objects) continue;
      const p = new THREE.Vector3();
      o.getWorldPosition(p);
      const d = p.distanceTo(here);
      if (d < bestD) {
        bestD = d;
        best = o;
      }
    }
    return best;
  }

  const lookAt = (target: Vec3, from: Vec3) => {
    controls.target.copy(toWorld(target));
    camera.position.copy(toWorld(from));
    controls.update();
  };

  const api: SceneApi = {
    armReady,
    debug() {
      const toolW = new THREE.Vector3();
      toolGroup.getWorldPosition(toolW);
      return {
        robotLoaded: !!robot,
        arm: deck.arm,
        balance: deck.balance,
        jointAngles: robot ? JOINT_NAMES.map((n) => robot!.joints[n]?.angle) : null,
        toolParent: toolGroup.parent?.name,
        toolWorld: toolW.toArray(),
        tcpWorld: tcpWorld().toArray(),
        held: held?.what ?? null,
        lastJoints,
        looseCap: { visible: looseCap.visible, parent: looseCap.parent === deckGroup ? 'deck' : looseCap.parent === tcp ? 'tcp' : 'none', pos: looseCap.position.toArray() },
        badGeometry: (() => {
          const bad: string[] = [];
          scene.traverse((o) => {
            const g = (o as THREE.Mesh).geometry;
            if (!g) return;
            if (!g.boundingSphere) g.computeBoundingSphere();
            if (g.boundingSphere && !Number.isFinite(g.boundingSphere.radius)) {
              const p = new THREE.Vector3();
              o.getWorldPosition(p);
              bad.push(`${g.type} at ${p.toArray().map((v) => v.toFixed(3)).join(',')} parent=${o.parent?.type} params=${JSON.stringify((g as THREE.CylinderGeometry).parameters)}`);
            }
          });
          return bad;
        })(),
      };
    },
    setJoints,
    setGripper: setFingers,
    attach(what) {
      if (held) return;
      if (what === 'cap') {
        const v = vialAt.capping_station;
        if (v && v.capOn) {
          v.setCap(false);
          looseCap.visible = true;
        } else if (!looseCap.visible || looseCap.parent !== objects) {
          return; // no cap to take
        }
        holdObject(looseCap, -0.006);
        held = { what, obj: looseCap };
        return;
      }
      if (what === 'tlh') {
        holdObject(tlh, 0);
        held = { what, obj: tlh };
      } else if (what === 'head') {
        const h = nearestTo(heads, 0.06);
        if (!h) return;
        if (h === dockedHead) dockedHead = null;
        holdObject(h, 0);
        held = { what, obj: h };
      } else {
        const v = nearestTo(Object.values(vials).map((x) => x.group), 0.07);
        if (!v) return;
        const vial = Object.values(vials).find((x) => x.group === v)!;
        noteVialAt(vial, 'tray');
        holdVial(vial);
      }
    },
    detach(what, at, slot) {
      if (!held || held.what !== what) return;
      if (what === 'cap') {
        if (at === 'vial') {
          vialAt.capping_station?.setCap(true);
          looseCap.visible = false;
          looseCap.removeFromParent();
          objects.add(looseCap);
        } else {
          const c = deck.capHolder.grab;
          park(looseCap, { ...c, z: c.z - 0.006 });
        }
      } else if (what === 'tlh') {
        park(tlh, deck.tlh.base.grab);
      } else if (what === 'head') {
        const m = at.match(/^head_rack_(\d+)$/);
        if (m && deck.dosingHeads) park(held.obj, deck.dosingHeads.rack[Number(m[1]) - 1].grab);
        else if (deck.dosingHeads) {
          park(held.obj, deck.dosingHeads.quantos.grab); // hangs upright on the hook
          dockedHead = held.obj;
        }
      } else if (held.vial) {
        if (deck.stations[at]?.virtual) {
          noteVialAt(held.vial, at); // "at" the station, but still in the gripper
          return;
        }
        putVial(held.vial, at, slot);
      }
      held = null;
    },
    station(station, action, args, progress, done) {
      const st = stations[station];
      if (!st) return;
      const v = vialAt[station];
      if (action === 'door_open' || action === 'door_close') {
        doorTargets.set(station, action === 'door_open' ? 1 : 0);
        return;
      }
      if (st.body) flashes.set(st.body, 1);
      if (station === 'capping_station') {
        // the jaws close on the vial and spin it (the arm holds the cap still), then let go
        const grip = done ? 0 : Math.min(1, progress * 4);
        const seat = vialSeat(st.def.location.grab, deck.vial);
        const gap = 0.035 - grip * (0.035 - deck.vial.radius - 0.011);
        st.extras.jawL.position.x = seat[0] - gap;
        st.extras.jawR.position.x = seat[0] + gap;
        if (v) v.group.rotation.z = done ? 0 : (action === 'uncap_vial' ? 1 : -1) * progress * 6 * Math.PI;
      } else if (station === 'analysis_station') {
        if (cameraLens) {
          const m = cameraLens.material as THREE.MeshStandardMaterial;
          m.emissiveIntensity = done ? 0.2 : 0.2 + 2.5 * Math.max(0, 1 - Math.abs(progress - 0.5) * 6);
        }
      } else if (station === 'solid_addition_station') {
        // the carrier lowers the gun onto the vial, powder flows, the carrier lifts it back
        const q = stations.weigh_balance;
        const hookZ = deck.dosingHeads?.quantos.grab.z ?? 0;
        const seat = vialSeat(st.def.location.grab, deck.vial);
        const lowered = Math.max(0, hookZ - (seat[2] + deck.vial.height + 0.07));
        const phase = done ? 0 : progress < 0.3 ? progress / 0.3 : progress > 0.7 ? (1 - progress) / 0.3 : 1;
        if (q?.extras.carrier) q.extras.carrier.position.z = hookZ + 0.02 - lowered * phase;
        if (dockedHead) dockedHead.position.z = hookZ - lowered * phase;
        if (q?.extras.stream) {
          q.extras.stream.visible = !done && progress > 0.3 && progress < 0.7;
          q.extras.stream.position.z = seat[2] + deck.vial.height;
        }
        if (done && v) v.setContents(v.solidMg + Number(args[0] ?? 0), v.liquidMl);
      } else if (station === 'liquid_addition_station') {
        if (v) {
          const target = Number(args[0] ?? 0);
          if (done) v.setContents(v.solidMg, v.liquidMl + target);
          else {
            const base = v.liquidMl;
            v.liquidMl = base + target * progress;
            v.render();
            v.liquidMl = base;
          }
        }
      } else if (station === 'weigh_balance') {
        if (done && st.valueLabel) {
          st.valueLabel.textContent = action === 'zero' ? '0.0 mg' : `${Number(args[0] ?? 0).toFixed(1)} mg`;
        }
      }
    },
    trace(tcpPose) {
      if (!trail.visible) return;
      const i = trailCount % TRAIL_MAX;
      trailPositions[i * 3] = tcpPose[3];
      trailPositions[i * 3 + 1] = tcpPose[7];
      trailPositions[i * 3 + 2] = tcpPose[11];
      trailCount++;
      if (trailCount <= TRAIL_MAX) {
        trailGeom.setDrawRange(0, trailCount);
      } else {
        const ordered = new Float32Array(TRAIL_MAX * 3);
        const start = trailCount % TRAIL_MAX;
        for (let k = 0; k < TRAIL_MAX; k++) {
          const src = (start + k) % TRAIL_MAX;
          ordered.set(trailPositions.subarray(src * 3, src * 3 + 3), k * 3);
        }
        trailPositions.set(ordered);
        trailCount = TRAIL_MAX;
        trailGeom.setDrawRange(0, TRAIL_MAX);
      }
      trailGeom.attributes.position.needsUpdate = true;
    },
    clearTrail() {
      trailCount = 0;
      trailGeom.setDrawRange(0, 0);
    },
    applyState(state) {
      // put every vial where the deck says it is; the held one (or one "at" a virtual station) onto the gripper
      const current = isTraySlot(state.current_vial) ? state.current_vial : null;
      if (held?.what === 'vial') {
        held.obj.removeFromParent();
        objects.add(held.obj);
        held = null;
      }
      for (let n = 1; n <= TRAY_SLOT_COUNT; n++) {
        const v = vials[n];
        const info = state.vial_contents?.[String(n)];
        if (n === current) {
          v.setContents(state.solid_added ?? 0, state.liquid_added ?? 0);
          v.setCap(state.cap_is_on ?? true);
          const loc = state.vial_location;
          if (state.is_held_by_arm) {
            noteVialAt(v, 'tray');
            holdVial(v);
          } else if (loc && loc !== 'tray' && deck.stations[loc]?.virtual) {
            noteVialAt(v, loc);
            holdVial(v);
          } else if (loc && loc !== 'tray') {
            putVial(v, loc);
          } else {
            putVial(v, 'tray', n);
          }
        } else {
          v.setContents(info?.processed ? info.solid_mg : 0, info?.processed ? info.liquid_ml : 0);
          v.setCap(info?.processed ? info.cap_is_on : true);
          putVial(v, 'tray', n);
        }
      }
      // the cap of an uncapped vial waits in the cap holder (unless the arm is carrying it)
      if (held?.what !== 'cap') {
        if (current && state.cap_is_on === false) {
          const c = deck.capHolder.grab;
          park(looseCap, { ...c, z: c.z - 0.006 });
          looseCap.visible = true;
        } else {
          looseCap.visible = false;
        }
      }
      if (held?.what !== 'tlh') park(tlh, deck.tlh.base.grab);
      if (held?.what !== 'head' && deck.dosingHeads) {
        heads.forEach((h, i) => park(h, deck.dosingHeads!.rack[i].grab));
        dockedHead = null;
      }
    },
    tick(dt) {
      for (const [mesh, k] of flashes) {
        const m = mesh.material as THREE.MeshStandardMaterial;
        m.emissive.set(0x38bdf8);
        m.emissiveIntensity = 0.35 * k;
        const next = k - dt * 1.5;
        if (next <= 0) {
          m.emissiveIntensity = 0;
          flashes.delete(mesh);
        } else {
          flashes.set(mesh, next);
        }
      }
      for (const [id, target] of doorTargets) {
        const d = stations[id]?.door;
        if (!d) continue;
        const open = (d.mesh.position.y - d.closedY) / d.travel;
        const next = open + Math.sign(target - open) * Math.min(Math.abs(target - open), dt * 1.6);
        d.mesh.position.y = d.closedY + next * d.travel;
        if (Math.abs(next - target) < 1e-4) doorTargets.delete(id);
      }
      controls.update();
      renderer.render(scene, camera);
      labelRenderer.render(scene, camera);
    },
    setSafePoints(visible) {
      safePoints.forEach((p) => (p.visible = visible));
    },
    setTrail(visible) {
      trail.visible = visible;
      if (!visible) api.clearTrail();
    },
    setLabels(visible) {
      labels.forEach((l) => (l.visible = visible));
    },
    setLocationsOnly(on) {
      objects.visible = !on;
      markers.visible = on;
      // CSS2D labels render whenever their object is in the scene, so hide them explicitly
      labels.forEach((l) => (l.visible = !on && (document.getElementById('chk-labels') as HTMLInputElement | null)?.checked !== false));
      markerLabels.forEach((l) => (l.visible = on));
    },
    resize() {
      const w = container.clientWidth || 1;
      const h = container.clientHeight || 1;
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      renderer.setSize(w, h);
      labelRenderer.setSize(w, h);
    },
    view(preset) {
      const cap = vialSeat(deck.stations.capping_station.location.grab, deck.vial);
      const bal = vialSeat(deck.stations.weigh_balance.location.grab, deck.vial);
      const tr = slotGrab(8, deck.tray);
      switch (preset) {
        case 'arm':
          lookAt([0.1, 0, 0.25], [0.9, 0.7, 0.75]);
          return;
        case 'balance':
          // from the robot's side of the Quantos (its door) and a little toward its front (the chamber)
          lookAt([bal[0], bal[1], bal[2] + 0.1], [bal[0] - 0.45, bal[1] + 0.32, bal[2] + 0.38]);
          return;
        case 'capping':
          // from beyond the table's far corner, so nothing stands between the camera and the capper
          lookAt([cap[0] - 0.02, cap[1] + 0.02, cap[2] + 0.04], [cap[0] - 0.38, cap[1] - 0.34, cap[2] + 0.34]);
          return;
        case 'tray':
          lookAt([tr.x, tr.y, tr.z - 0.02], [tr.x + 0.3, tr.y + 0.35, tr.z + 0.35]);
          return;
        default:
          lookAt([0.2, -0.15, 0.08], [1.15, 1.0, 1.05]);
      }
    },
  };

  api.setSafePoints(false);
  api.setTrail(false);
  api.resize();
  api.view('overview');
  setJoints(lastJoints);
  return api;
}
