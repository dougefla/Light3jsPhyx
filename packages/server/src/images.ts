import sharp from 'sharp';
import { PNG } from 'pngjs';
import { writeFile } from 'node:fs/promises';

/** Write top-down RGBA raw bytes as an RGB PNG. */
export async function writeRGBAPng(
  rgba: Buffer | Uint8Array,
  width: number,
  height: number,
  outPath: string,
): Promise<void> {
  await sharp(Buffer.from(rgba.buffer, rgba.byteOffset, rgba.byteLength), {
    raw: { width, height, channels: 4 },
  })
    .removeAlpha()
    .png({ compressionLevel: 6 })
    .toFile(outPath);
}

/**
 * Depth (meters, float32) → 16-bit grayscale PNG in millimeters
 * (RGB-D dataset convention: 0 = no hit, 1mm quantization, max 65.535m).
 */
export async function writeDepthPng16(
  depth: Float32Array,
  width: number,
  height: number,
  outPath: string,
): Promise<void> {
  const png = new PNG({ width, height, colorType: 0, inputColorType: 0, bitDepth: 16, inputHasAlpha: false });
  const data = Buffer.alloc(width * height * 2);
  for (let i = 0; i < depth.length; i++) {
    const mm = Math.max(0, Math.min(65535, Math.round(depth[i]! * 1000)));
    data.writeUInt16LE(mm, i * 2); // pngjs expects native-endian input and swaps to BE itself
  }
  png.data = data as unknown as Buffer;
  await writeFile(outPath, PNG.sync.write(png, { colorType: 0, inputColorType: 0, bitDepth: 16 }));
}

/** Lossless float32 export: minimal NumPy .npy (v1.0, C-order, shape [h, w]). */
export async function writeNpyFloat32(
  data: Float32Array,
  height: number,
  width: number,
  outPath: string,
): Promise<void> {
  let header = `{'descr': '<f4', 'fortran_order': False, 'shape': (${height}, ${width}), }`;
  const unpadded = 10 + header.length + 1;
  header = header + ' '.repeat((64 - (unpadded % 64)) % 64) + '\n';
  const head = Buffer.alloc(10 + header.length);
  head.write('\x93NUMPY', 0, 'latin1');
  head.writeUInt8(1, 6);
  head.writeUInt8(0, 7);
  head.writeUInt16LE(header.length, 8);
  head.write(header, 10, 'latin1');
  await writeFile(outPath, Buffer.concat([head, Buffer.from(data.buffer, data.byteOffset, data.byteLength)]));
}
