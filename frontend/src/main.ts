import './style.css';
import { Animator } from './animator';
import { choreograph } from './choreography';
import { buildDeck, isTraySlot, type BalanceModule } from './deck';
import { connectInput, type Input } from './events';
import { createScene, type ViewPreset } from './scene';
import { STOCK_SOLUTION_DEMO, Simulator, type SimEvent, type SimState } from './simulator';
import { Ui } from './ui';

const MODULE_KEY = 'beavers-bench-balance';

// Theme: the plugin bridge (bridge-client.js) sets `dark`/`light` on <html> from the IvoryOS UI around
// the page. Standalone, follow the operating system; there is no toggle, as on every plugin page.
function followSystemTheme() {
  const mq = window.matchMedia('(prefers-color-scheme: dark)');
  const apply = () => {
    if (window.IvoryBridge) return;
    document.documentElement.classList.toggle('dark', mq.matches);
    document.documentElement.classList.toggle('light', !mq.matches);
  };
  apply();
  mq.addEventListener('change', apply);
}

function savedModule(): BalanceModule {
  try {
    const v = localStorage.getItem(MODULE_KEY);
    if (v === 'balance' || v === 'quantos') return v;
  } catch {
    // storage blocked
  }
  return 'quantos';
}

async function boot() {
  followSystemTheme();
  const deck = buildDeck(savedModule());
  const container = document.getElementById('viewport')!;
  const scene = createScene(container, deck);

  let input: Input | null = null;
  let latestState: SimState | null = null;
  let slotHint: number | null = null;

  const ui = new Ui({
    onReset: () => {
      animator.clear();
      scene.clearTrail();
      input?.reset();
      ui.log('Reset', 'info');
    },
    onSpeed: (s) => (animator.speed = s),
    onSafePoints: (v) => scene.setSafePoints(v),
    onTrail: (v) => scene.setTrail(v),
    onLabels: (v) => scene.setLabels(v),
    onLocationsOnly: (v) => scene.setLocationsOnly(v),
    onView: (preset: ViewPreset) => scene.view(preset),
    onModule: (m: BalanceModule) => {
      try {
        localStorage.setItem(MODULE_KEY, m);
      } catch {
        // storage blocked: the choice lasts for this page only
      }
      location.reload(); // the deck is rebuilt from scratch; a live deck resyncs its state
    },
    onCall: (instrument, method, args) => {
      if (input?.mode !== 'sandbox') return;
      callSandbox(input.sim, instrument, method, args);
    },
    onDemo: () => runDemo(),
  }, deck);

  const animator = new Animator(scene, {
    onCallStart: (c) => {
      ui.startCall(c);
      ui.setPending(animator.pending);
    },
    onStep: (c, i) => ui.activeStep(c, i),
    onCallEnd: () => {
      ui.endCall();
      ui.setPending(animator.pending);
    },
    onIdle: () => {
      ui.setPending(0);
      if (latestState) scene.applyState(latestState); // settle on what the deck says
    },
    onWarning: (text) => ui.log(text, 'warn'),
  }, deck);

  const onEvent = (event: SimEvent) => {
    if (event.instrument === 'robotic_arm') {
      const n = Number(event.args?.[0]);
      if (event.action === 'pick_up_vial_from_tray' && isTraySlot(n)) slotHint = n;
    }
    const c = choreograph(event, slotHint, deck);
    if (c.steps.length === 0) {
      ui.log(`${c.call}${c.note ? `  (${c.note})` : ' (nothing to show)'}`, 'info');
      return;
    }
    if (c.note) ui.log(`${c.call}  (${c.note})`, 'info');
    animator.enqueue(c);
    ui.setPending(animator.pending);
  };

  input = await connectInput({
    onEvent,
    onState: (state) => {
      latestState = state;
      if (isTraySlot(state.current_vial)) slotHint = state.current_vial;
      ui.showState(state);
      if (!animator.busy) scene.applyState(state);
    },
    onConnection: (ok) => ui.setConnected(ok),
  });
  ui.setMode(input.mode);
  // a handle for poking at the page from the console
  (window as unknown as { __bench: unknown }).__bench = { animator, scene, input, deck };
  ui.log(input.mode === 'live' ? 'Showing the deck ivoryOS is running' : 'Sandbox: the simulator runs in this page', 'info');
  ui.log(`Deck: ${deck.arm}, ${deck.balance === 'quantos' ? 'Quantos (balance + powder dosing)' : 'weigh balance, no solid dosing'}`, 'info');
  scene.armReady.then(() => ui.log(`${deck.arm} model loaded`, 'info'));

  function callSandbox(sim: Simulator, instrument: string, method: string, args: unknown[]): boolean {
    const fn = (sim as unknown as Record<string, (...a: unknown[]) => unknown>)[method];
    if (typeof fn !== 'function') {
      ui.log(`${instrument}.${method}: no such method`, 'error');
      return false;
    }
    try {
      const result = fn.apply(sim, args);
      if (typeof result === 'number') ui.log(`${instrument}.${method}() -> ${result.toFixed(3)}`, 'info');
      return true;
    } catch (err) {
      ui.log(`RuntimeError: ${(err as Error).message}`, 'error');
      return false;
    }
  }

  let demoRunning = false;
  async function runDemo() {
    if (input?.mode !== 'sandbox' || demoRunning) return;
    demoRunning = true;
    const sim = input.sim;
    const VIAL = Number((document.getElementById('slot') as HTMLInputElement)?.value || 1);
    const SOLID_MG = Number((document.getElementById('mass') as HTMLInputElement)?.value || 25.5);
    const LIQUID_ML = Number((document.getElementById('volume') as HTMLInputElement)?.value || 15);
    const sleep = (s: number) => new Promise((r) => setTimeout(r, (s * 1000) / Math.max(animator.speed, 0.1)));
    try {
      for (const line of STOCK_SOLUTION_DEMO) {
        const m = line.match(/^(\w+)\.(\w+)\((.*)\)$/);
        if (!m) continue;
        const [, instrument, method, argText] = m;
        const args: unknown[] = [];
        if (argText) {
          const value = argText.split('=').pop()!.trim();
          args.push(value === 'VIAL' ? VIAL : value === 'SOLID_MG' ? SOLID_MG : value === 'LIQUID_ML' ? LIQUID_ML : Number(value));
        }
        if (!callSandbox(sim, instrument, method, args)) break;
        await animator.drain();
        if (method === 'start_stirring') await sleep(1);
      }
    } finally {
      demoRunning = false;
    }
  }

  // render loop
  let last = performance.now();
  const frame = (now: number) => {
    const dt = Math.min(0.1, (now - last) / 1000);
    last = now;
    animator.tick(dt);
    scene.tick(dt);
    requestAnimationFrame(frame);
  };
  requestAnimationFrame(frame);
  new ResizeObserver(() => scene.resize()).observe(container);
}

boot().catch((err) => {
  console.error(err);
  const el = document.getElementById('log-list');
  if (el) el.innerHTML = `<li class="error">${(err as Error).message}</li>`;
});
