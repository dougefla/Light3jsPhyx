import RAPIER from '@dimforge/rapier3d-compat';

let ready: Promise<typeof RAPIER> | null = null;

/** Initialize the Rapier WASM module (idempotent, safe to call often). */
export function initRapier(): Promise<typeof RAPIER> {
  // (the "deprecated parameters" console warning comes from inside the
  // compat package's own wasm bootstrap, not from this call)
  ready ??= RAPIER.init().then(() => RAPIER);
  return ready;
}

export { RAPIER };
