/**
 * Wire protocol shared by pages, server and SDKs. Pure data helpers only —
 * this module must stay importable from Node (no three.js / DOM).
 *
 * Control frames: JSON text `{id, method, params}` → `{id, result | error}`.
 * Binary frames:  [u32 LE request id][u8 channel][payload…]
 */

export const CHANNEL = { rgb: 1, seg: 2, depth: 3, video: 4 } as const;
export type ChannelName = keyof typeof CHANNEL;

export interface RpcRequest {
  id: number;
  method: string;
  params?: Record<string, unknown>;
}

export interface RpcResponse {
  id: number;
  result?: unknown;
  error?: { message: string };
}

export const BINARY_HEADER_BYTES = 5;

export function encodeBinaryFrame(
  requestId: number,
  channel: number,
  payload: Uint8Array,
): Uint8Array<ArrayBuffer> {
  const out = new Uint8Array(new ArrayBuffer(BINARY_HEADER_BYTES + payload.byteLength));
  new DataView(out.buffer).setUint32(0, requestId, true);
  out[4] = channel;
  out.set(payload, BINARY_HEADER_BYTES);
  return out;
}

/**
 * Allocate a frame with the header pre-filled and return the payload view, so
 * a producer (GPU readback) can write straight into the frame — no second copy
 * of a multi-megabyte pass before it hits the socket.
 */
export function allocBinaryFrame(
  requestId: number,
  channel: number,
  payloadLength: number,
): { frame: Uint8Array<ArrayBuffer>; payload: Uint8Array<ArrayBuffer> } {
  const frame = new Uint8Array(new ArrayBuffer(BINARY_HEADER_BYTES + payloadLength));
  new DataView(frame.buffer).setUint32(0, requestId, true);
  frame[4] = channel;
  return { frame, payload: frame.subarray(BINARY_HEADER_BYTES) };
}

export function decodeBinaryFrame(data: Uint8Array): {
  requestId: number;
  channel: number;
  payload: Uint8Array;
} {
  const view = new DataView(data.buffer, data.byteOffset, data.byteLength);
  return {
    requestId: view.getUint32(0, true),
    channel: data[4]!,
    payload: data.subarray(BINARY_HEADER_BYTES),
  };
}

/** 24-bit segmentation id encoding (id 0 = background). */
export function idToRGB(id: number): [number, number, number] {
  return [(id >> 16) & 0xff, (id >> 8) & 0xff, id & 0xff];
}

export function rgbToId(r: number, g: number, b: number): number {
  return (r << 16) | (g << 8) | b;
}
