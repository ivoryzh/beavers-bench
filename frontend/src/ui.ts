// The panel beside the scene: what is being called, the ladder of arm moves it expands into, the
// activity log, and (sandbox only) the instrument methods as buttons.
import type { Choreography } from './choreography';
import { DECK, TRAY_SLOT_COUNT, type BalanceModule, type DeckDef } from './deck';
import type { ViewPreset } from './scene';
import type { SimState } from './simulator';

export interface UiHandlers {
  onReset(): void;
  onSpeed(speed: number): void;
  onSafePoints(visible: boolean): void;
  onTrail(visible: boolean): void;
  onLabels(visible: boolean): void;
  onLocationsOnly(on: boolean): void;
  onView(preset: ViewPreset): void;
  onModule(module: BalanceModule): void;
  onCall(instrument: string, method: string, args: unknown[]): void;
  onDemo(): void;
}

const $ = <T extends HTMLElement>(id: string) => document.getElementById(id) as T;

export class Ui {
  private ladder = $<HTMLOListElement>('ladder');
  private ladderCall = $<HTMLDivElement>('ladder-call');
  private logList = $<HTMLUListElement>('log-list');
  private status = $<HTMLDivElement>('connection-status');
  private modeChip = $<HTMLDivElement>('mode-chip');
  private deckState = $<HTMLDivElement>('deck-state');
  private queued = $<HTMLSpanElement>('queued');
  private controls = $<HTMLDivElement>('controls');
  private stepItems: HTMLLIElement[] = [];

  constructor(private h: UiHandlers, private deck: DeckDef = DECK) {
    $('btn-reset').addEventListener('click', () => h.onReset());
    for (const b of document.querySelectorAll<HTMLButtonElement>('button[data-view]')) {
      b.addEventListener('click', () => h.onView(b.dataset.view as ViewPreset));
    }
    for (const r of document.querySelectorAll<HTMLInputElement>('input[name="module"]')) {
      r.checked = r.value === deck.balance;
      r.addEventListener('change', () => {
        if (r.checked) h.onModule(r.value as BalanceModule);
      });
    }
    const speed = $<HTMLInputElement>('speed');
    const speedVal = $<HTMLSpanElement>('speed-val');
    speed.addEventListener('input', () => {
      speedVal.textContent = `${Number(speed.value).toFixed(1)}×`;
      h.onSpeed(Number(speed.value));
    });
    $<HTMLInputElement>('chk-safe').addEventListener('change', (e) => h.onSafePoints((e.target as HTMLInputElement).checked));
    $<HTMLInputElement>('chk-trail').addEventListener('change', (e) => h.onTrail((e.target as HTMLInputElement).checked));
    $<HTMLInputElement>('chk-labels').addEventListener('change', (e) => h.onLabels((e.target as HTMLInputElement).checked));
    $<HTMLInputElement>('chk-locations').addEventListener('change', (e) => h.onLocationsOnly((e.target as HTMLInputElement).checked));
    $('deck-name').textContent = DECK.name;
  }

  setMode(mode: 'live' | 'sandbox') {
    this.modeChip.textContent = mode === 'live' ? 'Driven by ivoryOS' : 'Sandbox';
    this.modeChip.classList.toggle('live', mode === 'live');
    this.controls.hidden = mode === 'live';
    if (mode === 'sandbox') this.buildControls();
  }

  setConnected(connected: boolean) {
    this.status.textContent = connected ? 'Connected' : '···';
    this.status.classList.toggle('connected', connected);
  }

  setPending(n: number) {
    this.queued.textContent = n > 1 ? `${n - 1} queued` : '';
  }

  showState(s: SimState) {
    const where = s.is_held_by_arm ? 'in the gripper' : s.vial_location === 'tray' ? 'in the tray' : `on ${s.vial_location}`;
    const vial = s.current_vial ? `Vial ${s.current_vial} ${where}` : 'No vial out';
    const bits = [vial];
    if (s.current_vial && (s.is_held_by_arm || s.vial_location !== 'tray')) {
      bits.push(s.cap_is_on ? 'capped' : 'uncapped');
      if (s.solid_added) bits.push(`${s.solid_added} mg solid`);
      if (s.liquid_added) bits.push(`${s.liquid_added} mL liquid`);
    }
    this.deckState.textContent = bits.join(' · ');
  }

  startCall(c: Choreography) {
    this.ladderCall.textContent = c.call;
    this.ladder.innerHTML = '';
    this.stepItems = [];
    for (const step of c.steps) {
      if (!step.code) continue;
      const li = document.createElement('li');
      li.dataset.kind = step.kind;
      const tag = document.createElement('span');
      tag.className = 'tag';
      tag.textContent = step.kind === 'movej' ? 'movej' : step.kind === 'movel' ? 'movel' : step.kind;
      li.appendChild(tag);
      li.appendChild(document.createTextNode(step.code));
      this.ladder.appendChild(li);
      this.stepItems.push(li);
      (li as HTMLLIElement & { stepIndex?: number }).stepIndex = c.steps.indexOf(step);
    }
    this.log(c.call, 'call');
  }

  activeStep(c: Choreography, index: number) {
    for (const li of this.stepItems) {
      const i = (li as HTMLLIElement & { stepIndex?: number }).stepIndex ?? -1;
      li.classList.toggle('active', i === index);
      li.classList.toggle('done', i < index);
    }
    const step = c.steps[index];
    if (step && (step.kind === 'movej' || step.kind === 'movel')) {
      const li = this.stepItems.find((l) => (l as HTMLLIElement & { stepIndex?: number }).stepIndex === index);
      li?.scrollIntoView({ block: 'nearest' });
    }
  }

  endCall() {
    for (const li of this.stepItems) {
      li.classList.remove('active');
      li.classList.add('done');
    }
  }

  log(text: string, kind: 'info' | 'call' | 'error' | 'warn' = 'info') {
    const li = document.createElement('li');
    li.className = kind;
    li.textContent = `[${new Date().toLocaleTimeString()}] ${text}`;
    this.logList.prepend(li);
    while (this.logList.children.length > 80) this.logList.removeChild(this.logList.lastChild!);
  }

  private buildControls() {
    this.controls.innerHTML = '';
    const group = (title: string, note?: string) => {
      const sec = document.createElement('section');
      const h = document.createElement('h3');
      h.textContent = title;
      sec.appendChild(h);
      if (note) {
        const p = document.createElement('p');
        p.className = 'note';
        p.textContent = note;
        sec.appendChild(p);
      }
      const row = document.createElement('div');
      row.className = 'btn-row';
      sec.appendChild(row);
      this.controls.appendChild(sec);
      return row;
    };
    const button = (row: HTMLElement, text: string, onClick: () => void, cls = '') => {
      const b = document.createElement('button');
      b.className = `method ${cls}`.trim();
      b.textContent = text;
      b.addEventListener('click', onClick);
      row.appendChild(b);
      return b;
    };
    const number = (row: HTMLElement, id: string, labelText: string, value: number, step = 1, min?: number, max?: number) => {
      const wrap = document.createElement('label');
      wrap.className = 'num';
      wrap.textContent = labelText;
      const inp = document.createElement('input');
      inp.type = 'number';
      inp.id = id;
      inp.value = String(value);
      inp.step = String(step);
      if (min !== undefined) inp.min = String(min);
      if (max !== undefined) inp.max = String(max);
      wrap.appendChild(inp);
      row.appendChild(wrap);
      return inp;
    };

    const demo = group('Workflow', 'The stock-solution demo from scripts/demo_prepare_stock_solution_complete.py, call by call.');
    button(demo, 'Run stock-solution demo', () => this.h.onDemo(), 'primary');

    const arm = group('robotic_arm', 'Every method of the simulated RoboticArm, by its Python name.');
    const slot = number(arm, 'slot', 'vial_number', 1, 1, 1, TRAY_SLOT_COUNT);
    button(arm, 'pick_up_vial_from_tray(n)', () => this.h.onCall('robotic_arm', 'pick_up_vial_from_tray', [Number(slot.value)]));
    button(arm, 'place_vial_in_tray(n)', () => this.h.onCall('robotic_arm', 'place_vial_in_tray', [Number(slot.value)]));
    const places: [string, string][] = [
      ['weigh_balance', 'on'], ['capping_station', 'on'], ['solid_addition_station', 'on'],
      ['liquid_addition_station', 'on'], ['stir_plate', 'on'], ['analysis_station', 'in'],
    ];
    for (const [st, prep] of places) {
      button(arm, `place_vial_${prep}_${st}()`, () => this.h.onCall('robotic_arm', `place_vial_${prep}_${st}`, []));
      button(arm, `pick_up_vial_${prep}_${st}()`, () => this.h.onCall('robotic_arm', `pick_up_vial_${prep}_${st}`, []));
    }

    const cap = group('capping_station');
    button(cap, 'uncap_vial()', () => this.h.onCall('capping_station', 'uncap_vial', []));
    button(cap, 'cap_vial()', () => this.h.onCall('capping_station', 'cap_vial', []));

    const bal = group('weigh_balance', this.deck.balance === 'quantos' ? 'The Quantos balance: the vial is weighed where it stands.' : 'The balance with its vial holder.');
    button(bal, 'zero()', () => this.h.onCall('weigh_balance', 'zero', []));
    button(bal, 'get_weight_mg()', () => this.h.onCall('weigh_balance', 'get_weight_mg', []));

    const solid = group('solid_addition_station', this.deck.dosingHeads
      ? 'The arm carries a dosing head from the rack into the Quantos, which doses the vial.'
      : 'No solid dosing on this bench: the calls run, the vial stays in the gripper.');
    const mg = number(solid, 'mass', 'mass_mg', 25.5, 0.5, 0);
    button(solid, 'add_solid(mass_mg)', () => this.h.onCall('solid_addition_station', 'add_solid', [Number(mg.value)]));

    const liquid = group('liquid_addition_station', 'The arm fetches the liquid handler (TLH) from its base station and doses the vial where it stands in the capping station.');
    const ml = number(liquid, 'volume', 'volume_ml', 10, 0.5, 0);
    button(liquid, 'add_liquid(volume_ml)', () => this.h.onCall('liquid_addition_station', 'add_liquid', [Number(ml.value)]));

    const stir = group('stir_plate', 'No stir plate on this deck: start_stirring makes the arm invert the vial it holds.');
    button(stir, 'start_stirring()', () => this.h.onCall('stir_plate', 'start_stirring', []));
    button(stir, 'stop_stirring()', () => this.h.onCall('stir_plate', 'stop_stirring', []));

    const ana = group('sample_analysis_station', 'No analysis station on this deck yet.');
    button(ana, 'analyze()', () => this.h.onCall('sample_analysis_station', 'analyze', []));
  }
}
