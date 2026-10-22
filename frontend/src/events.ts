// Where the events come from: the ivoryos-next plugin bridge when the page is served by the plugin
// (bridge-client.js exists next to it), otherwise the in-page simulator (sandbox mode).
import { Simulator, type SimEvent, type SimState } from './simulator';

export interface BridgeSocket {
  on(event: string, cb: (data?: unknown) => void): BridgeSocket;
  emit(event: string): BridgeSocket;
}

declare global {
  interface Window {
    IvoryBridge?: { connect(): BridgeSocket; theme(): string };
  }
}

export interface InputHandlers {
  onEvent(event: SimEvent): void;
  onState(state: SimState): void;
  onConnection(connected: boolean): void;
}

export type Input =
  | { mode: 'live'; reset(): void }
  | { mode: 'sandbox'; sim: Simulator; reset(): void };

function loadBridgeClient(): Promise<boolean> {
  if (window.IvoryBridge) return Promise.resolve(true);
  if (location.protocol === 'file:') return Promise.resolve(false);
  return new Promise((resolve) => {
    const s = document.createElement('script');
    s.src = new URL('bridge-client.js', location.href).toString();
    s.onload = () => resolve(!!window.IvoryBridge);
    s.onerror = () => resolve(false);
    document.head.appendChild(s);
  });
}

export async function connectInput(h: InputHandlers): Promise<Input> {
  if (await loadBridgeClient()) {
    const socket = window.IvoryBridge!.connect();
    socket.on('connect', () => h.onConnection(true));
    socket.on('disconnect', () => h.onConnection(false));
    socket.on('sync_state', (state) => h.onState(state as SimState));
    socket.on('instrument_event', (event) => h.onEvent(event as SimEvent));
    socket.on('replay_events', () => {});
    return { mode: 'live', reset: () => socket.emit('reset_state') };
  }
  const sim = new Simulator();
  sim.onEvent = (e) => h.onEvent(e);
  sim.onState = (s) => h.onState(s);
  h.onConnection(true);
  h.onState(sim.build_sim_state());
  return { mode: 'sandbox', sim, reset: () => sim.reset_state() };
}
