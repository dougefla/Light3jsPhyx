import { describe, it, expect } from 'vitest';
import {
  CHANNEL,
  encodeBinaryFrame,
  decodeBinaryFrame,
  idToRGB,
  rgbToId,
} from '@lightphyx/engine';

describe('binary framing', () => {
  it('round-trips request id, channel and payload', () => {
    const payload = new Uint8Array([1, 2, 3, 250, 251]);
    const frame = encodeBinaryFrame(0xdeadbeef >>> 0, CHANNEL.depth, payload);
    const decoded = decodeBinaryFrame(frame);
    expect(decoded.requestId).toBe(0xdeadbeef >>> 0);
    expect(decoded.channel).toBe(CHANNEL.depth);
    expect([...decoded.payload]).toEqual([...payload]);
  });

  it('handles empty payloads', () => {
    const frame = encodeBinaryFrame(7, CHANNEL.video, new Uint8Array(0));
    expect(decodeBinaryFrame(frame).payload.byteLength).toBe(0);
  });
});

describe('segmentation id encoding', () => {
  it('round-trips every byte boundary', () => {
    for (const id of [0, 1, 255, 256, 65535, 65536, 0xffffff]) {
      const [r, g, b] = idToRGB(id);
      expect(rgbToId(r, g, b)).toBe(id);
    }
  });

  it('id 0 is pure black (background)', () => {
    expect(idToRGB(0)).toEqual([0, 0, 0]);
  });
});
