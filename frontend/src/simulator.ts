// A TypeScript port of scripts/instruments.py + scripts/sim_instruments.py: the same instrument and
// method names, the same checks and error messages, the same events and state shape. It is what runs
// the page in sandbox mode (no Python, no ivoryOS); with ivoryOS the Python simulator is the source
// of truth and this file is not used.
import { TRAY_SLOT_COUNT } from './deck';

export interface SimEvent {
  instrument: string;
  action: string;
  args: unknown[];
  kwargs?: Record<string, unknown>;
}

export interface VialContents { solid_mg: number; liquid_ml: number; cap_is_on: boolean; processed: boolean }

/** The dict build_sim_state() returns on the Python side. */
export interface SimState {
  vial_location: string | null;
  is_held_by_arm: boolean;
  solid_added: number;
  liquid_added: number;
  cap_is_on: boolean;
  current_vial: number | null;
  vial_contents: Record<string, VialContents>;
}

const DEFAULT_VIAL_WEIGHT_MG = 1500;

const emptyContents = (): VialContents => ({ solid_mg: 0, liquid_ml: 0, cap_is_on: true, processed: false });

export class Simulator {
  private vialLocation: string | null = 'tray';
  private isHeldByArm = false;
  private capIsOn = true;
  private currentVial: number | null = null;
  private nextAvailableVial = 1;
  private vialContents: Record<number, VialContents> = {};
  private dummyVialWeightMg = DEFAULT_VIAL_WEIGHT_MG;
  private solidAddedMg = 0;
  private liquidAddedMl = 0;
  private tareMg = 0;
  stirring = false;

  readonly density_g_ml = 1.0;
  onEvent: (event: SimEvent) => void = () => {};
  onState: (state: SimState) => void = () => {};

  constructor() {
    this.reset_state();
  }

  reset_state(): void {
    this.vialLocation = 'tray';
    this.isHeldByArm = false;
    this.capIsOn = true;
    this.currentVial = null;
    this.nextAvailableVial = 1;
    this.vialContents = {};
    for (let i = 1; i <= TRAY_SLOT_COUNT; i++) this.vialContents[i] = emptyContents();
    this.dummyVialWeightMg = DEFAULT_VIAL_WEIGHT_MG;
    this.solidAddedMg = 0;
    this.liquidAddedMl = 0;
    this.tareMg = 0;
    this.stirring = false;
    this.onState(this.build_sim_state());
  }

  build_sim_state(): SimState {
    const contents: Record<string, VialContents> = {};
    for (const [k, v] of Object.entries(this.vialContents)) contents[k] = { ...v };
    return {
      vial_location: this.vialLocation,
      is_held_by_arm: this.isHeldByArm,
      solid_added: this.solidAddedMg,
      liquid_added: this.liquidAddedMl,
      cap_is_on: this.capIsOn,
      current_vial: this.currentVial,
      vial_contents: contents,
    };
  }

  private emit(instrument: string, action: string, ...args: unknown[]) {
    this.onEvent({ instrument, action, args, kwargs: {} });
  }

  private changed() {
    this.onState(this.build_sim_state());
  }

  // ── RoboticArm ────────────────────────────────────────────────────────────────────────────────
  pick_up_vial_from_tray(vial_number: number | null = null): void {
    if (this.isHeldByArm) throw new Error('Robotic arm is already holding a vial');
    if (this.vialLocation !== 'tray') throw new Error(`Vial is not in the tray, it is at: ${this.vialLocation}`);
    let n: number;
    if (vial_number === null || vial_number === undefined) {
      n = this.nextAvailableVial;
      this.nextAvailableVial += 1;
    } else {
      n = Math.trunc(Number(vial_number));
      this.nextAvailableVial = Math.max(this.nextAvailableVial, n + 1);
    }
    this.emit('robotic_arm', 'pick_up_vial_from_tray', n);
    this.isHeldByArm = true;
    this.vialLocation = null;
    this.currentVial = n;
    this.dummyVialWeightMg = DEFAULT_VIAL_WEIGHT_MG;
    this.solidAddedMg = 0;
    this.liquidAddedMl = 0;
    this.capIsOn = true;
    this.changed();
  }

  place_vial_in_tray(vial_number: number | null = null): void {
    if (!this.isHeldByArm) throw new Error('Robotic arm is not holding a vial');
    const n = vial_number === null || vial_number === undefined ? this.currentVial : vial_number;
    const slot = Math.trunc(Number(n));
    if (this.currentVial !== null && slot !== this.currentVial) {
      throw new Error(
        `Cannot place vial in slot ${slot}: arm is holding vial from slot ${this.currentVial}. ` +
          'Vials must be returned to their original slot.',
      );
    }
    this.emit('robotic_arm', 'place_vial_in_tray', slot);
    this.vialContents[slot] = {
      solid_mg: this.solidAddedMg,
      liquid_ml: this.liquidAddedMl,
      cap_is_on: this.capIsOn,
      processed: true,
    };
    this.isHeldByArm = false;
    this.vialLocation = 'tray';
    this.currentVial = slot;
    this.changed();
  }

  private pickUpAt(location: string, action: string, where: string) {
    if (this.isHeldByArm) throw new Error('Robotic arm is already holding a vial');
    if (this.vialLocation !== location) throw new Error(`Vial is not ${where}, it is at: ${this.vialLocation}`);
    this.emit('robotic_arm', action);
    this.isHeldByArm = true;
    this.vialLocation = null;
    this.changed();
  }

  private placeAt(location: string, action: string) {
    if (!this.isHeldByArm) throw new Error('Robotic arm is not holding a vial');
    this.emit('robotic_arm', action);
    this.isHeldByArm = false;
    this.vialLocation = location;
    this.changed();
  }

  pick_up_vial_on_weigh_balance() { this.pickUpAt('weigh_balance', 'pick_up_vial_on_weigh_balance', 'on the weigh balance'); }
  place_vial_on_weigh_balance() { this.placeAt('weigh_balance', 'place_vial_on_weigh_balance'); }
  pick_up_vial_on_stir_plate() { this.pickUpAt('stir_plate', 'pick_up_vial_on_stir_plate', 'on the stir plate'); }
  place_vial_on_stir_plate() { this.placeAt('stir_plate', 'place_vial_on_stir_plate'); }
  pick_up_vial_on_capping_station() { this.pickUpAt('capping_station', 'pick_up_vial_on_capping_station', 'on the capping station'); }
  place_vial_on_capping_station() { this.placeAt('capping_station', 'place_vial_on_capping_station'); }
  pick_up_vial_on_solid_addition_station() { this.pickUpAt('solid_addition_station', 'pick_up_vial_on_solid_addition_station', 'on the solid addition station'); }
  place_vial_on_solid_addition_station() { this.placeAt('solid_addition_station', 'place_vial_on_solid_addition_station'); }
  pick_up_vial_on_liquid_addition_station() { this.pickUpAt('liquid_addition_station', 'pick_up_vial_on_liquid_addition_station', 'on the liquid addition station'); }
  place_vial_on_liquid_addition_station() { this.placeAt('liquid_addition_station', 'place_vial_on_liquid_addition_station'); }
  pick_up_vial_in_analysis_station() { this.pickUpAt('analysis_station', 'pick_up_vial_in_analysis_station', 'in the analysis station'); }
  place_vial_in_analysis_station() { this.placeAt('analysis_station', 'place_vial_in_analysis_station'); }

  // ── WeighBalance ──────────────────────────────────────────────────────────────────────────────
  zero(): void {
    this.emit('weigh_balance', 'zero');
    this.tareMg = this.vialLocation === 'weigh_balance' ? this.dummyVialWeightMg : 0;
  }

  get_weight_mg(): number {
    const net = (this.vialLocation === 'weigh_balance' ? this.dummyVialWeightMg : 0) - this.tareMg;
    this.emit('weigh_balance', 'get_weight_mg', net);
    return net;
  }

  // ── StirPlate ─────────────────────────────────────────────────────────────────────────────────
  start_stirring(): void { this.emit('stir_plate', 'start_stirring'); this.stirring = true; }
  stop_stirring(): void { this.emit('stir_plate', 'stop_stirring'); this.stirring = false; }

  // ── CappingStation ────────────────────────────────────────────────────────────────────────────
  uncap_vial(): void {
    if (this.vialLocation !== 'capping_station') {
      throw new Error(`Cannot uncap vial if it is not on the capping station. Current location: ${this.vialLocation}`);
    }
    if (!this.capIsOn) throw new Error('Vial is already uncapped');
    this.emit('capping_station', 'uncap_vial');
    this.capIsOn = false;
    this.changed();
  }

  cap_vial(): void {
    if (this.vialLocation !== 'capping_station') {
      throw new Error(`Cannot cap vial if it is not on the capping station. Current location: ${this.vialLocation}`);
    }
    if (this.capIsOn) throw new Error('Vial is already capped');
    this.emit('capping_station', 'cap_vial');
    this.capIsOn = true;
    this.changed();
  }

  // ── SolidAdditionStation ──────────────────────────────────────────────────────────────────────
  add_solid(mass_mg: number): void {
    if (this.vialLocation !== 'solid_addition_station') {
      throw new Error(`Cannot add solid if vial is not on the solid addition station. Current location: ${this.vialLocation}`);
    }
    if (this.capIsOn) throw new Error('Cannot add solid because the vial cap is on');
    this.emit('solid_addition_station', 'add_solid', mass_mg);
    this.dummyVialWeightMg += mass_mg;
    this.solidAddedMg += mass_mg;
    this.changed();
  }

  // ── LiquidAdditionStation ─────────────────────────────────────────────────────────────────────
  add_liquid(volume_ml: number, density_g_ml: number | null = null): void {
    if (this.vialLocation !== 'liquid_addition_station') {
      throw new Error(`Cannot add liquid if vial is not on the liquid addition station. Current location: ${this.vialLocation}`);
    }
    if (this.capIsOn) throw new Error('Cannot add liquid because the vial cap is on');
    const density = density_g_ml ?? this.density_g_ml;
    this.emit('liquid_addition_station', 'add_liquid', volume_ml);
    this.dummyVialWeightMg += volume_ml * density * 1000;
    this.liquidAddedMl += volume_ml;
    this.changed();
  }

  // ── SampleAnalysisStation ─────────────────────────────────────────────────────────────────────
  analyze(input_1: number | null = null, input_2: number | null = null): number {
    if (this.vialLocation !== 'analysis_station') {
      throw new Error(`Cannot analyze if vial is not on the analysis station. Current location: ${this.vialLocation}`);
    }
    const x = input_1 ?? -5 + Math.random() * 15;
    const y = input_2 ?? Math.random() * 15;
    const value = branin(x, y) + 0.05 * gaussian();
    this.emit('sample_analysis_station', 'analyze', value);
    return value;
  }
}

function branin(x: number, y: number): number {
  const a = 1, b = 5.1 / (4 * Math.PI ** 2), c = 5 / Math.PI, r = 6, s = 10, t = 1 / (8 * Math.PI);
  return a * (y - b * x * x + c * x - r) ** 2 + s * (1 - t) * Math.cos(x) + s;
}

function gaussian(): number {
  const u = 1 - Math.random(), v = Math.random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

/** The instrument-level calls of scripts/demo_prepare_stock_solution_complete.py, one per line, as text. */
export const STOCK_SOLUTION_DEMO = [
  'weigh_balance.zero()',
  'robotic_arm.pick_up_vial_from_tray(vial_number=VIAL)',
  'robotic_arm.place_vial_on_capping_station()',
  'capping_station.uncap_vial()',
  'robotic_arm.pick_up_vial_on_capping_station()',
  'robotic_arm.place_vial_on_weigh_balance()',
  'weigh_balance.get_weight_mg()',
  'robotic_arm.pick_up_vial_on_weigh_balance()',
  'robotic_arm.place_vial_on_solid_addition_station()',
  'solid_addition_station.add_solid(SOLID_MG)',
  'robotic_arm.pick_up_vial_on_solid_addition_station()',
  'robotic_arm.place_vial_on_weigh_balance()',
  'weigh_balance.get_weight_mg()',
  'robotic_arm.pick_up_vial_on_weigh_balance()',
  'robotic_arm.place_vial_on_liquid_addition_station()',
  'liquid_addition_station.add_liquid(LIQUID_ML)',
  'robotic_arm.pick_up_vial_on_liquid_addition_station()',
  'robotic_arm.place_vial_on_weigh_balance()',
  'weigh_balance.get_weight_mg()',
  'robotic_arm.pick_up_vial_on_weigh_balance()',
  'robotic_arm.place_vial_on_capping_station()',
  'capping_station.cap_vial()',
  'robotic_arm.pick_up_vial_on_capping_station()',
  'robotic_arm.place_vial_on_stir_plate()',
  'stir_plate.start_stirring()',
  'stir_plate.stop_stirring()',
  'robotic_arm.pick_up_vial_on_stir_plate()',
  'robotic_arm.place_vial_in_tray(vial_number=VIAL)',
];
