// The scene's points are a hand transcription of deck/locations_education_deck.yaml into deck.ts.
// This test reads the yaml itself and checks every transcribed number against it, so the
// "locations only" view and the choreography really show the file's values.
import { readFileSync } from 'node:fs';
import { join } from 'node:path';
import { load } from 'js-yaml';
import { describe, expect, it } from 'vitest';
import { DECK, FILE_POINTS, QUANTOS_FILE_HOOK, QUANTOS_SHIFT, TRAY_PITCH, TRAY_SHIFT, heldAtPads, type Pose } from '../src/deck';

const doc = load(readFileSync(join(__dirname, '..', 'deck', 'locations_education_deck.yaml'), 'utf8')) as Record<string, any>;

const samePose = (p: Pose, y: { x: number; y: number; z: number; rz: number }, name: string) => {
  expect(p.x * 1000, `${name} x`).toBeCloseTo(y.x, 6);
  expect(p.y * 1000, `${name} y`).toBeCloseTo(y.y, 6);
  expect(p.z * 1000, `${name} z`).toBeCloseTo(y.z, 6);
  expect(p.rz, `${name} rz`).toBe(y.rz);
};

const point = (name: string) => FILE_POINTS.find((p) => p.name === name)!.pose;

describe('deck.ts against locations_education_deck.yaml', () => {
  it('homes: TCP poses and joint angles', () => {
    for (const side of ['positive_x', 'positive_y', 'negative_y'] as const) {
      samePose(point(`arm_home.${side}`), doc.arm_home[side], `arm_home.${side}`);
      const joints = doc.arm_home[`joints_${side}`] as number[];
      DECK.homes[side].forEach((q, i) => {
        // deck.ts keeps the angles in radians, wrist 3 shifted by -360 degrees (same angle)
        const degrees = ((q * 180) / Math.PI + 720) % 360;
        expect(degrees, `${side} joint ${i}`).toBeCloseTo((joints[i] + 720) % 360, 6);
      });
    }
  });

  it('trays 1-5: the three corner poses, rows and columns', () => {
    for (let n = 1; n <= 5; n++) {
      const tr = doc.vial_at_vial_tray[`tray_${n}`];
      samePose(point(`tray_${n}.grab`), tr.grab, `tray_${n}.grab`);
      samePose(point(`tray_${n}.bottom_left`), tr.bottom_left, `tray_${n}.bottom_left`);
      samePose(point(`tray_${n}.bottom_right`), tr.bottom_right, `tray_${n}.bottom_right`);
    }
    // the drawn tray is the file's tray_2 slid TRAY_SHIFT along the rack, clear of the dosing heads, with its
    // slots spread to TRAY_PITCH so the gripper's fingers fit between the vials
    const t2 = doc.vial_at_vial_tray.tray_2;
    const unshift = (p: Pose): Pose => ({ ...p, x: p.x - TRAY_SHIFT[0], y: p.y - TRAY_SHIFT[1], z: p.z - TRAY_SHIFT[2] });
    samePose(unshift(DECK.tray.grab), t2.grab, 'DECK.tray.grab');
    expect(DECK.tray.grab.x - DECK.tray.bottomLeft.x).toBeCloseTo((t2.rows - 1) * TRAY_PITCH, 9);
    expect(DECK.tray.bottomLeft.y - DECK.tray.bottomRight.y).toBeCloseTo((t2.columns - 1) * TRAY_PITCH, 9);
    expect(TRAY_PITCH).toBeGreaterThan(2 * DECK.vial.radius + 0.008); // room for a finger between two vials
    expect(DECK.tray.rows).toBe(t2.rows);
    expect(DECK.tray.cols).toBe(t2.columns);
    expect(DECK.tray.approach * 1000).toBe(t2.approach.z);
    expect(DECK.tray.safe.y! * 1000).toBe(t2.safe.y);
    expect(DECK.tray.safe.z! * 1000).toBe(t2.safe.z);
    expect(DECK.tray.gripperOpen).toBe(doc.gripper_positions[t2.gripper_open]);
  });

  it('capping station, cap holder, Quantos, dosing heads, TLH', () => {
    const cap = doc.vial_at_capping_station.vial_16_ml;
    samePose(DECK.stations.capping_station.location.grab, cap.grab, 'capping');
    expect(DECK.stations.capping_station.location.approach * 1000).toBe(cap.approach.z);
    expect(DECK.stations.capping_station.location.safe.y! * 1000).toBe(cap.safe.y);
    expect(DECK.capping.capZ * 1000).toBe(cap.capping_z_offset);
    expect(DECK.capping.uncapZ * 1000).toBe(cap.uncapping_z_offset);
    samePose(DECK.capHolder.grab, doc.vial_cap_at_cap_holder.grab, 'cap holder');

    // the Quantos vial seat is deliberately NOT the file's vial_at_quantos (the bench has the Quantos
    // elsewhere); it sits right under the dosing-head hook instead, and the file's point is still listed
    const q = doc.vial_at_quantos.vial_16_ml;
    samePose(point('vial_at_quantos'), q.grab, 'vial_at_quantos (file point)');
    expect(DECK.stations.weigh_balance.location.approach * 1000).toBe(q.approach.z);
    expect(DECK.stations.weigh_balance.location.gripperOpen).toBe(doc.gripper_positions[q.gripper_open]);
    // the head hangs where the gripper's pads hold it, a little in front of the hook TCP point; the vial holder is under that
    const head = heldAtPads(DECK.dosingHeads!.quantos.grab);
    expect(DECK.stations.weigh_balance.location.grab.x).toBeCloseTo(head.x, 6);
    expect(DECK.stations.weigh_balance.location.grab.y).toBeCloseTo(head.y, 6);

    for (let i = 1; i <= 8; i++) {
      samePose(DECK.dosingHeads!.rack[i - 1].grab, doc.dosing_head_at_rack[`dosing_head_${i}`].grab, `dosing_head_${i}`);
    }
    // the whole Quantos, hook included, is slid QUANTOS_SHIFT toward the robot; the file's point is still listed
    expect(DECK.stations.weigh_balance.location.safe.y).toBeLessThan(-0.15); // the arm comes in and out sideways, clear of the door
    const hq = doc.dosing_head_at_quantos;
    samePose(point('dosing_head_at_quantos'), hq.grab, 'dosing_head_at_quantos (file point)');
    samePose(QUANTOS_FILE_HOOK, hq.grab, 'QUANTOS_FILE_HOOK');
    const g = DECK.dosingHeads!.quantos.grab;
    expect([g.x - QUANTOS_SHIFT[0], g.y - QUANTOS_SHIFT[1], g.z - QUANTOS_SHIFT[2]].map((v) => +(v * 1000).toFixed(3))).toEqual([hq.grab.x, hq.grab.y, hq.grab.z]);
    expect(g.rz).toBe(hq.grab.rz);
    expect(DECK.dosingHeads!.quantos.safe.y! * 1000).toBe(hq.safe.y);
    expect(DECK.dosingHeads!.quantos.bufferSafe!.y! * 1000).toBe(hq.buffer_safe.y);
    expect(DECK.dosingHeads!.quantos.bufferSafe!.z! * 1000).toBe(hq.buffer_safe.z);

    const tlh = doc.tlh_at_base_station;
    samePose(DECK.tlh.base.grab, tlh.grab, 'tlh base');
    expect(DECK.tlh.base.approach * 1000).toBe(tlh.approach.z);
    expect(DECK.tlh.base.safe.y! * 1000).toBe(tlh.safe.y);
    const rel = doc.tlh_at_capping_station.vial_16_ml;
    expect(DECK.tlh.doseOffset.map((v) => v * 1000)).toEqual([rel.grab.x, rel.grab.y, rel.grab.z]);
    expect(DECK.tlh.doseApproach * 1000).toBe(rel.approach.z);
  });

  it('gripper TCP and openings', () => {
    const tcp = doc.grippers.universal_gripper.tcp as number[];
    expect(DECK.tcp.xyz).toEqual(tcp.slice(0, 3));
    expect(DECK.tcp.rx).toBeCloseTo(tcp[3], 4);
    expect(DECK.gripper.open).toBe(doc.gripper_positions.open);
    expect(DECK.gripper.closed).toBe(doc.gripper_positions.closed);
  });

  it('every file point drawn in the "locations only" view is in the file, and nothing in the file is missing', () => {
    const fileNames = [
      ...['positive_x', 'positive_y', 'negative_y'].map((s) => `arm_home.${s}`),
      ...[1, 2, 3, 4, 5].flatMap((n) => ['grab', 'bottom_left', 'bottom_right'].map((c) => `tray_${n}.${c}`)),
      'vial_at_capping_station', 'vial_cap_at_cap_holder', 'tlh_at_base_station', 'tlh_at_capping_station (relative)',
      'vial_at_quantos', 'dosing_head_at_quantos',
      ...Object.keys(doc.dosing_head_at_rack),
    ];
    expect(FILE_POINTS.map((p) => p.name).sort()).toEqual(fileNames.sort());
  });
});
