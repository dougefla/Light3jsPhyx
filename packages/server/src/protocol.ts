/**
 * Wire protocol framing — MUST stay in sync with
 * packages/engine/src/protocol/shared.ts (duplicated so the Node build
 * never has to compile browser-package TS sources).
 */

export const CHANNEL = { rgb: 1, seg: 2, depth: 3, video: 4 } as const;

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

export function encodeBinaryFrame(requestId: number, channel: number, payload: Uint8Array): Buffer {
  const head = Buffer.alloc(BINARY_HEADER_BYTES);
  head.writeUInt32LE(requestId, 0);
  head.writeUInt8(channel, 4);
  return Buffer.concat([head, payload]);
}

export function decodeBinaryFrame(data: Buffer): {
  requestId: number;
  channel: number;
  payload: Buffer;
} {
  return {
    requestId: data.readUInt32LE(0),
    channel: data.readUInt8(4),
    payload: data.subarray(BINARY_HEADER_BYTES),
  };
}
