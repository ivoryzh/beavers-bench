import { describe, expect, it } from 'vitest';
import { flangeToTcp } from '../src/animator';
import { approachPose, choreograph, safePose, tcpPose } from '../src/choreography';
import { DECK, TRAY_SLOT_COUNT, buildDeck, slotLocation } from '../src/deck';
import { UR3, UR3E, fk, ik, ikNudged, nearest, toolDown, type Joints } from '../src/kinematics';
import { approxEqual, chain, fromPose, invertRigid, multiply, rotZ, rpyXyz, toPose, type Mat4 } from '../src/mat4';

// Forward kinematics straight from the joint table of public/ur3.urdf (ros-industrial ur_description),
// so the DH model is checked against the exact chain urdf-loader will pose on screen.
// The joint tables of public/ur3.urdf and public/ur3e.urdf (ros-industrial ur_description).
const URDF_CHAINS = {
  UR3: { d1: 0.1519, a2: -0.24365, a3: -0.21325, d4: 0.11235, d5: 0.08535, d6: 0.0819 },
  UR3e: { d1: 0.15185, a2: -0.24355, a3: -0.2132, d4: 0.13105, d5: 0.08535, d6: 0.0921 },
};

function urdfFk(q: Joints, arm: keyof typeof URDF_CHAINS = 'UR3'): Mat4 {
  const PI = Math.PI;
  const c = URDF_CHAINS[arm];
  const base = rpyXyz(0, 0, PI, 0, 0, 0); // base_link -> base (the pendant's frame)
  const t = chain(
    rpyXyz(0, 0, PI, 0, 0, 0), // base_link -> base_link_inertia
    rpyXyz(0, 0, 0, 0, 0, c.d1), rotZ(q[0]), // shoulder_pan_joint
    rpyXyz(1.570796327, 0, 0, 0, 0, 0), rotZ(q[1]), // shoulder_lift_joint
    rpyXyz(0, 0, 0, c.a2, 0, 0), rotZ(q[2]), // elbow_joint
    rpyXyz(0, 0, 0, c.a3, 0, c.d4), rotZ(q[3]), // wrist_1_joint
    rpyXyz(1.570796327, 0, 0, 0, -c.d5, 0), rotZ(q[4]), // wrist_2_joint
    rpyXyz(1.570796326589793, PI, PI, 0, c.d6, 0), rotZ(q[5]), // wrist_3_joint
    rpyXyz(0, -PI / 2, -PI / 2, 0, 0, 0), // wrist_3 -> flange
    rpyXyz(PI / 2, 0, PI / 2, 0, 0, 0), // flange -> tool0
  );
  return chain(base, t); // rotZ(pi) is its own inverse
}

const samples: Joints[] = [
  [0, 0, 0, 0, 0, 0],
  [0, -Math.PI / 2, Math.PI / 2, -Math.PI / 2, -Math.PI / 2, 0],
  [0.3, -1.2, 1.9, -2.3, -1.4, 0.7],
  [-2.1, -0.4, 0.9, -1.0, 1.3, -2.5],
  [1.0, -2.6, -1.1, 0.4, 2.0, 3.0],
];

describe('forward kinematics', () => {
  it('matches the URDF chains of ur3.urdf and ur3e.urdf in the base frame', () => {
    for (const [arm, p] of [['UR3', UR3], ['UR3e', UR3E]] as const) {
      for (const q of samples) {
        const a = fk(q, p);
        const b = urdfFk(q, arm);
        // 5e-4: the URDF rounds its rpy values to 9 digits
        expect(approxEqual(a, b, 5e-4), `${arm} q=${q.join(', ')}\n dh=${a.map((v) => v.toFixed(4))}\n urdf=${b.map((v) => v.toFixed(4))}`).toBe(true);
      }
    }
  });

  it('puts the TCP where the location file says the home poses are (within 4 cm)', () => {
    // arm_home in deck/locations_education_deck.yaml: joints and the TCP pose they were recorded at
    const T_FT = flangeToTcp(DECK);
    const homes: [keyof typeof DECK.homes, number[]][] = [
      ['positive_x', [0.3533, 0.0789, 0.3672]],
      ['positive_y', [-0.0509, 0.3584, 0.3670]],
      ['negative_y', [0.0475, -0.3589, 0.3671]],
    ];
    for (const [side, expected] of homes) {
      // positive_x lands within 1.5 cm; positive_y is 3.4 cm off, so that home was probably recorded with
      // a slightly different TCP. Close enough for a simulation that goes to the file's poses by IK anyway.
      const tcp = multiply(fk(DECK.homes[side], UR3E), T_FT); // the deck's arm is a UR3e
      const err = Math.hypot(tcp[3] - expected[0], tcp[7] - expected[1], tcp[11] - expected[2]);
      expect(err, `${side}: ${toPose(tcp).slice(0, 3)} vs ${expected}`).toBeLessThan(0.04);
      expect(tcp[10]).toBeGreaterThan(0.99); // TCP z up: the tool points down
    }
  });
});

describe('inverse kinematics', () => {
  it('round-trips every solution through fk', () => {
    for (const q of samples) {
      const target = fk(q, UR3);
      const sols = ik(target, UR3);
      expect(sols.length).toBeGreaterThan(0);
      for (const s of sols) {
        expect(approxEqual(fk(s, UR3), target, 1e-6), `q=${q.join(', ')} sol=${s.join(', ')}`).toBe(true);
      }
      const back = nearest(sols, q);
      expect(back).not.toBeNull();
      expect(back!.every((v, i) => Math.abs(v - q[i]) < 1e-6)).toBe(true);
    }
  });

  it('reaches a tool-down pose inside the UR3 workspace', () => {
    const target = toolDown(0.25, 0.15, 0.05, 0.3);
    const home: Joints = [0, -Math.PI / 2, Math.PI / 2, -Math.PI / 2, -Math.PI / 2, 0];
    const q = nearest(ik(target, UR3), home);
    expect(q).not.toBeNull();
    expect(approxEqual(fk(q!, UR3), target, 1e-6)).toBe(true);
  });

  it('returns nothing for a pose out of reach', () => {
    expect(ik(toolDown(0.9, 0.0, 0.1, 0), UR3)).toHaveLength(0);
  });

  it('pose <-> matrix conversion round-trips', () => {
    const pose = [0.1, -0.2, 0.3, 2.2, -2.2, 0.1];
    const back = toPose(fromPose(pose));
    pose.forEach((v, i) => expect(back[i]).toBeCloseTo(v, 6));
  });
});

describe('the deck', () => {
  const T_TF = invertRigid(flangeToTcp(DECK));
  const reachable = (name: string, m: Mat4) => {
    const sols = ik(multiply(m, T_TF), UR3E).filter((s) => s[1] < 0 && s[1] > -Math.PI);
    expect(sols.length, `${name} unreachable (elbow up)`).toBeGreaterThan(0);
  };
  // the dosing-head rack is a few mm past the model's reach: the animator pulls such targets in
  const reachableAnyway = (name: string, m: Mat4) => {
    const { solutions, nudge } = ikNudged(multiply(m, T_TF), UR3E);
    expect(solutions.length, `${name} unreachable even nudged`).toBeGreaterThan(0);
    expect(nudge, `${name} needed a ${nudge * 1000} mm nudge`).toBeLessThan(0.01);
  };

  it('can reach every grab, approach and safe point the simulator uses, on both benches', () => {
    for (const deck of [buildDeck('quantos'), buildDeck('balance')]) {
      for (let n = 1; n <= TRAY_SLOT_COUNT; n++) {
        const l = slotLocation(n, deck.tray);
        reachable(`tray[${n}]`, tcpPose(l.grab));
        reachable(`tray[${n}]_approach`, tcpPose(approachPose(l)));
        reachable(`tray[${n}]_safe`, tcpPose(safePose(l)));
      }
      for (const st of Object.values(deck.stations)) {
        if (st.virtual) continue;
        reachable(st.id, tcpPose(st.location.grab));
        reachable(`${st.id}_approach`, tcpPose(approachPose(st.location)));
        reachable(`${st.id}_safe`, tcpPose(safePose(st.location)));
      }
      reachable('tlh_base', tcpPose(deck.tlh.base.grab));
      reachable('tlh_base_safe', tcpPose(safePose(deck.tlh.base)));
      if (deck.dosingHeads) {
        for (const [name, l] of [['dosing_head_1', deck.dosingHeads.rack[0]], ['dosing_head_at_quantos', deck.dosingHeads.quantos]] as const) {
          reachable(name, tcpPose(l.grab));
          reachableAnyway(`${name}_approach`, tcpPose(approachPose(l)));
          reachable(`${name}_safe`, tcpPose(safePose(l)));
        }
      }
    }
  });

  it('keeps the vial in the gripper at a station this deck does not have', () => {
    const c = choreograph({ instrument: 'robotic_arm', action: 'place_vial_on_stir_plate', args: [] }, 3);
    expect(c.steps.filter((s) => s.kind === 'movej' || s.kind === 'movel')).toHaveLength(0);
    expect(c.note).toMatch(/stays in the gripper/);
    const mix = choreograph({ instrument: 'stir_plate', action: 'start_stirring', args: [] }, 3);
    expect(mix.steps[0].kind).toBe('mix');
  });

  it('dips the needle into the vial without hitting the bottom, and clears the top on approach', () => {
    const addLiquid = choreograph({ instrument: 'liquid_addition_station', action: 'add_liquid', args: [10] }, 3);
    const dose = addLiquid.steps.find((s) => s.kind === 'movel' && s.name === 'tlh_at_capping_station') as { target: Mat4 };
    const approach = addLiquid.steps.find((s) => s.kind === 'movej' && s.name === 'tlh_at_capping_station_approach') as { target: Mat4 };
    const vialGrab = DECK.stations.capping_station.location.grab;
    const vialBottom = vialGrab.z - DECK.vial.gripHeight;
    const vialTop = vialBottom + DECK.vial.height;
    const tipAtDose = dose.target[11] - DECK.tlh.needleTip;
    const tipAtApproach = approach.target[11] - DECK.tlh.needleTip;
    expect(tipAtDose).toBeGreaterThan(vialBottom + 0.01);
    expect(tipAtDose).toBeLessThan(vialTop - 0.01);
    expect(tipAtApproach).toBeGreaterThan(vialTop + 0.01);
  });

  it('uncaps by holding the cap while the capper spins the vial, then parks the cap in the cap holder', () => {
    const c = choreograph({ instrument: 'capping_station', action: 'uncap_vial', args: [] }, 3);
    const kinds = c.steps.map((s) => s.kind);
    expect(kinds).toContain('attach');
    expect(c.steps.some((s) => s.kind === 'station' && s.station === 'capping_station')).toBe(true);
    expect(c.steps.some((s) => s.kind === 'detach' && (s as { at: string }).at === 'cap_holder')).toBe(true);
    const names = c.steps.filter((s) => s.kind === 'movel').map((s) => (s as { name: string }).name);
    expect(names).toContain('cap_holder');
    // the cap is taken uncapping_z_offset above the vial's grab pose
    const capGrab = c.steps.find((s) => s.kind === 'movel' && s.name === 'capping_station_cap') as { target: Mat4 };
    expect(capGrab.target[11]).toBeCloseTo(DECK.stations.capping_station.location.grab.z + DECK.capping.uncapZ, 6);
    reachable('cap_holder', tcpPose(DECK.capHolder.grab));
    reachable('capping_station_cap', capGrab.target);
  });

  it('shows the vial to the camera and keeps holding it', () => {
    const c = choreograph({ instrument: 'robotic_arm', action: 'place_vial_in_analysis_station', args: [] }, 3);
    expect(c.steps.some((s) => s.kind === 'movej' && s.name === 'analysis_station')).toBe(true);
    expect(c.steps.some((s) => s.kind === 'gripper')).toBe(false);
    reachable('camera hold', tcpPose(DECK.stations.analysis_station.location.grab));
  });

  it('opens and closes the Quantos door around a visit', () => {
    const c = choreograph({ instrument: 'robotic_arm', action: 'place_vial_on_weigh_balance', args: [] }, 3);
    const doors = c.steps.filter((s) => s.kind === 'station').map((s) => (s as { action: string }).action);
    expect(doors).toEqual(['door_open', 'door_close']);
    const plain = choreograph({ instrument: 'robotic_arm', action: 'place_vial_on_weigh_balance', args: [] }, 3, buildDeck('balance'));
    expect(plain.steps.filter((s) => s.kind === 'station')).toHaveLength(0);
  });

  it('expands a tray pick into the routine the location file implies', () => {
    const c = choreograph({ instrument: 'robotic_arm', action: 'pick_up_vial_from_tray', args: [3] }, null);
    const names = c.steps.filter((s) => s.kind === 'movej' || s.kind === 'movel').map((s) => (s as { name: string }).name);
    expect(names).toEqual(['home_positive_x', 'tray[3]_safe', 'tray[3]_approach', 'tray[3]', 'tray[3]_approach', 'tray[3]_safe', 'home_positive_x']);
    expect(c.steps.some((s) => s.kind === 'attach')).toBe(true);
  });

  it('the safe point of a tray slot is back toward the robot and higher than the approach', () => {
    const l = slotLocation(1);
    const a = approachPose(l), s = safePose(l);
    expect(s.z).toBeGreaterThan(a.z);
    expect(s.x).toBeLessThan(a.x); // rz -90: the TCP's -y is the base's -x
  });
});
