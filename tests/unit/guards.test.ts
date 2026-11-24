import { describe, it, expect } from 'vitest';
import { Env } from '@lightphyx/engine';

// Issue #24: non-finite joint values / coordinates silently poison scene
// matrices (black render, NaN bodies) with no error. The engine boundary now
// rejects them. These guards run before any robot/physics lookup, so they can
// be exercised on a bare Env.
describe('finiteness guards', () => {
  it('setJoint rejects inf / -inf / NaN before anything else', () => {
    const env = new Env();
    for (const bad of [Infinity, -Infinity, NaN]) {
      expect(() => env.setJoint('anything', bad)).toThrow(/finite/);
    }
  });

  it('setJoint passes the finiteness check for a finite value', () => {
    const env = new Env();
    // Finite value clears the guard; it then fails on "No robot loaded",
    // proving the guard did not fire for a legitimate value.
    expect(() => env.setJoint('anything', -0.1)).toThrow(/No robot loaded/);
  });

  it('spawn rejects non-finite pos or size', () => {
    const env = new Env();
    expect(() => env.spawn({ shape: 'sphere', size: [0.1], pos: [NaN, 0, 0] })).toThrow(/finite/);
    expect(() => env.spawn({ shape: 'sphere', size: [Infinity], pos: [0, 0, 0] })).toThrow(/finite/);
  });
});
