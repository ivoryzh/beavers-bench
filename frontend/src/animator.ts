// Plays choreographies on the arm: joint-space moves (movej), straight-line moves (movel, inverse
// kinematics every frame), gripper and station actions, one step at a time, as the real robot would.
// Targets are TCP poses; the gripper's TCP offset (deck.tcp) turns them into flange poses for the IK.
import type { Choreography, Held, Step } from './choreography';
import { DECK, type DeckDef, type DeckPosition } from './deck';
import { UR3, UR3E, fk, ik, ikNudged, jointDistance, nearest, type Joints, type URParams } from './kinematics';
import { chain, interpolate, invertRigid, multiply, rotX, translate, type Mat4 } from './mat4';

export interface ArmView {
  setJoints(q: Joints): void;
  setGripper(open: number): void; // 0 closed .. 1 fully open
  attach(what: Held): void;
  detach(what: Held, at: string, slot?: number): void;
  station(station: string, action: string, args: unknown[], progress: number, done: boolean): void;
  trace(tcp: Mat4): void;
}

export interface AnimatorEvents {
  onCallStart(c: Choreography): void;
  onStep(c: Choreography, index: number): void;
  onCallEnd(c: Choreography): void;
  onIdle(): void;
  onWarning(text: string): void;
}

const JOINT_SPEED = 1.8; // rad/s, the pace of a movej at speed 1
const LINEAR_SPEED = 0.28; // m/s, a movel
const GRIPPER_SECONDS = 0.35;

const easeInOut = (t: number) => (t < 0.5 ? 2 * t * t : 1 - Math.pow(-2 * t + 2, 2) / 2);

/** flange -> TCP for the deck's gripper. */
export function flangeToTcp(deck: DeckDef): Mat4 {
  const [x, y, z] = deck.tcp.xyz;
  return chain(translate(x, y, z), rotX(deck.tcp.rx));
}

export class Animator {
  joints: Joints;
  gripper = 1;
  speed = 1;
  private queue: Choreography[] = [];
  private current: Choreography | null = null;
  private stepIndex = -1;
  private elapsed = 0;
  private seconds = 0;
  private fromJoints: Joints;
  private toJoints: Joints | null = null;
  private fromPose: Mat4 | null = null;
  private toPose: Mat4 | null = null;
  private fromGrip = 1;
  private toGrip = 1;
  private idleNotified = true;
  private waiters: (() => void)[] = [];
  private params: URParams;
  private T_FT: Mat4;
  private T_TF: Mat4;

  constructor(private view: ArmView, private events: AnimatorEvents, private deck: DeckDef = DECK) {
    this.params = deck.arm === 'UR3e' ? UR3E : UR3;
    this.T_FT = flangeToTcp(deck);
    this.T_TF = invertRigid(this.T_FT);
    this.joints = [...deck.homes.positive_x] as Joints;
    this.fromJoints = [...this.joints] as Joints;
    view.setJoints(this.joints);
    view.setGripper(1);
  }

  get busy(): boolean {
    return this.current !== null || this.queue.length > 0;
  }

  get pending(): number {
    return this.queue.length + (this.current ? 1 : 0);
  }

  enqueue(c: Choreography): void {
    if (c.steps.length === 0) return;
    this.queue.push(c);
    this.idleNotified = false;
  }

  /** Resolves once everything queued has played. */
  drain(): Promise<void> {
    if (!this.busy) return Promise.resolve();
    return new Promise((resolve) => this.waiters.push(resolve));
  }

  /** Drop everything, put the arm home, open the gripper (a Reset). */
  clear(): void {
    this.queue = [];
    this.current = null;
    this.stepIndex = -1;
    this.toJoints = null;
    this.toPose = null;
    this.joints = [...this.deck.homes.positive_x] as Joints;
    this.gripper = 1;
    this.view.setJoints(this.joints);
    this.view.setGripper(1);
    this.notifyIdle();
  }

  /** The TCP pose now. */
  tcp(): Mat4 {
    return multiply(fk(this.joints, this.params), this.T_FT);
  }

  /** Advance by dt seconds of wall time. Call every frame. */
  tick(dt: number): void {
    // keep up with a Python deck that runs ahead: play faster the longer the queue
    const factor = this.speed * (1 + 0.35 * Math.max(0, this.pending - 1));
    let budget = dt * factor;
    let guard = 0;
    while (budget > 0 && guard++ < 50) {
      if (!this.current) {
        const next = this.queue.shift();
        if (!next) {
          this.notifyIdle();
          return;
        }
        this.current = next;
        this.stepIndex = -1;
        this.events.onCallStart(next);
        this.beginNextStep();
        continue;
      }
      const step = this.current.steps[this.stepIndex];
      const left = this.seconds - this.elapsed;
      const use = Math.min(left, budget);
      this.elapsed += use;
      budget -= use;
      this.applyStep(step, this.seconds > 0 ? Math.min(1, this.elapsed / this.seconds) : 1);
      if (this.elapsed >= this.seconds - 1e-9) {
        this.finishStep(step);
        this.beginNextStep();
      }
    }
  }

  private notifyIdle() {
    if (this.idleNotified) return;
    this.idleNotified = true;
    const w = this.waiters;
    this.waiters = [];
    w.forEach((f) => f());
    this.events.onIdle();
  }

  private beginNextStep(): void {
    if (!this.current) return;
    for (;;) {
      this.stepIndex += 1;
      if (this.stepIndex >= this.current.steps.length) {
        const done = this.current;
        this.current = null;
        this.events.onCallEnd(done);
        return;
      }
      const step = this.current.steps[this.stepIndex];
      this.events.onStep(this.current, this.stepIndex);
      this.elapsed = 0;
      this.seconds = 0;
      if (this.startStep(step)) return; // instantaneous steps fall through to the next one
    }
  }

  /** Returns false when the step is instantaneous and already done. */
  private startStep(step: Step): boolean {
    switch (step.kind) {
      case 'movej': {
        const target = typeof step.target === 'string'
          ? ([...this.deck.homes[step.target as DeckPosition]] as Joints)
          : this.solve(step.target, step.name);
        if (!target) return false;
        this.fromJoints = [...this.joints] as Joints;
        this.toJoints = target;
        this.seconds = Math.max(0.35, jointDistance(this.fromJoints, target) / JOINT_SPEED);
        return true;
      }
      case 'movel': {
        const target = this.solve(step.target, step.name);
        if (!target) return false;
        this.fromPose = this.tcp();
        this.toPose = step.target;
        this.fromJoints = [...this.joints] as Joints;
        const d = Math.hypot(step.target[3] - this.fromPose[3], step.target[7] - this.fromPose[7], step.target[11] - this.fromPose[11]);
        this.seconds = Math.max(0.25, d / LINEAR_SPEED);
        return true;
      }
      case 'gripper':
        this.fromGrip = this.gripper;
        this.toGrip = step.open ? Math.max(0.15, step.width / this.deck.gripper.open) : 0;
        this.seconds = GRIPPER_SECONDS;
        return true;
      case 'attach':
        this.view.attach(step.what);
        return false;
      case 'detach':
        this.view.detach(step.what, step.at, step.slot);
        return false;
      case 'station':
        this.seconds = step.seconds;
        this.view.station(step.station, step.action, step.args, 0, false);
        return true;
      case 'mix':
        this.fromJoints = [...this.joints] as Joints;
        this.seconds = step.seconds;
        return true;
      case 'wait':
        this.seconds = step.seconds;
        return true;
    }
  }

  private applyStep(step: Step, t: number): void {
    switch (step.kind) {
      case 'movej': {
        if (!this.toJoints) return;
        const e = easeInOut(t);
        this.joints = this.fromJoints.map((v, i) => v + (this.toJoints![i] - v) * e) as Joints;
        this.view.setJoints(this.joints);
        this.view.trace(this.tcp());
        return;
      }
      case 'movel': {
        if (!this.fromPose || !this.toPose) return;
        const pose = interpolate(this.fromPose, this.toPose, easeInOut(t));
        const q = nearest(ik(multiply(pose, this.T_TF), this.params), this.joints);
        if (q) {
          this.joints = q;
          this.view.setJoints(q);
          this.view.trace(pose);
        }
        return;
      }
      case 'gripper': {
        this.gripper = this.fromGrip + (this.toGrip - this.fromGrip) * t;
        this.view.setGripper(this.gripper);
        return;
      }
      case 'station':
        this.view.station(step.station, step.action, step.args, t, false);
        return;
      case 'mix': {
        // wrist 2 turns the gripper over and back: 0 -> 180 deg -> 0, `cycles` times
        const swing = Math.PI * Math.sin(Math.PI * step.cycles * t) ** 2;
        this.joints = [...this.fromJoints] as Joints;
        this.joints[4] = this.fromJoints[4] + swing;
        this.view.setJoints(this.joints);
        this.view.trace(this.tcp());
        return;
      }
      default:
        return;
    }
  }

  private finishStep(step: Step): void {
    if (step.kind === 'movej' && this.toJoints) {
      this.joints = [...this.toJoints] as Joints;
      this.view.setJoints(this.joints);
    } else if (step.kind === 'movel' && this.toPose) {
      const q = nearest(ik(multiply(this.toPose, this.T_TF), this.params), this.joints);
      if (q) {
        this.joints = q;
        this.view.setJoints(q);
      }
    } else if (step.kind === 'station') {
      this.view.station(step.station, step.action, step.args, 1, true);
    } else if (step.kind === 'mix') {
      this.joints = [...this.fromJoints] as Joints;
      this.view.setJoints(this.joints);
    }
  }

  /** Joints for a TCP pose, closest to where the arm is: elbow up when it can, any way at the edge of reach. */
  private solve(tcpTarget: Mat4, name: string): Joints | null {
    const flange = multiply(tcpTarget, this.T_TF);
    const { solutions, nudge } = ikNudged(flange, this.params);
    const q = nearest(solutions, this.joints, (s) => s[1] < 0 && s[1] > -Math.PI) ?? nearest(solutions, this.joints);
    if (q && nudge > 0) this.events.onWarning(`${name}: ${(nudge * 1000).toFixed(0)} mm past the ${this.params.name} model's reach, pulled in that much`);
    if (!q) {
      this.events.onWarning(`${name}: out of reach for the ${this.params.name} (TCP ${tcpTarget[3].toFixed(3)}, ${tcpTarget[7].toFixed(3)}, ${tcpTarget[11].toFixed(3)}), skipping the move`);
      return null;
    }
    return q;
  }
}
