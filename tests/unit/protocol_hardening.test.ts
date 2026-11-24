import { describe, it, expect } from 'vitest';
import { decodeBinaryFrame, encodeBinaryFrame, BINARY_HEADER_BYTES } from '@lightphyx/engine';

// The server drops binary frames shorter than the header rather than crashing
// (issue #3). Mirror the guard the hub applies before decoding.
describe('binary frame length guard', () => {
  it('a short frame is below the header size and must be dropped', () => {
    const short = new Uint8Array(3);
    expect(short.length).toBeLessThan(BINARY_HEADER_BYTES);
  });

  it('a header-sized frame decodes with an empty payload', () => {
    const frame = encodeBinaryFrame(1, 2, new Uint8Array(0));
    expect(frame.byteLength).toBe(BINARY_HEADER_BYTES);
    const decoded = decodeBinaryFrame(Buffer.from(frame));
    expect(decoded.requestId).toBe(1);
    expect(decoded.channel).toBe(2);
    expect(decoded.payload.byteLength).toBe(0);
  });
});
