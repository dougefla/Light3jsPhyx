import http from 'node:http';
import { createReadStream, existsSync, statSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const MIME: Record<string, string> = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript',
  '.mjs': 'text/javascript',
  '.css': 'text/css',
  '.json': 'application/json',
  '.png': 'image/png',
  '.jpg': 'image/jpeg',
  '.obj': 'text/plain',
  '.mtl': 'text/plain',
  '.urdf': 'application/xml',
  '.wasm': 'application/wasm',
  '.svg': 'image/svg+xml',
};

export function webuiDistDir(): string {
  return path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../../webui/dist');
}

function send(res: http.ServerResponse, filePath: string, headOnly = false): void {
  if (!existsSync(filePath)) {
    res.writeHead(404).end('not found');
    return;
  }
  const stat = statSync(filePath);
  if (!stat.isFile()) {
    res.writeHead(404).end('not found');
    return;
  }
  // last-modified + content-length let the in-page mesh cache detect a
  // file rewritten at the same path (issue: stale geometry after re-export).
  res.writeHead(200, {
    'content-type': MIME[path.extname(filePath).toLowerCase()] ?? 'application/octet-stream',
    'cache-control': 'no-store',
    'content-length': String(stat.size),
    'last-modified': stat.mtime.toUTCString(),
  });
  if (headOnly) {
    res.end();
    return;
  }
  createReadStream(filePath).pipe(res);
}

/**
 * Static handler: serves the built WebUI at /, and local files under /files/
 * (the bridge that lets pool pages fetch URDF/mesh assets from disk).
 * This is a local tool — the server binds to 127.0.0.1 by default.
 */
export function createStaticHandler(distDir: string) {
  return (req: http.IncomingMessage, res: http.ServerResponse): void => {
    const url = new URL(req.url ?? '/', 'http://localhost');
    let p = decodeURIComponent(url.pathname);
    if (p === '/favicon.ico') {
      res.writeHead(204).end();
      return;
    }
    if (p.startsWith('/files/')) {
      send(res, path.resolve('/', p.slice('/files/'.length)), req.method === 'HEAD');
      return;
    }
    if (p === '/') p = '/index.html';
    const resolved = path.resolve(distDir, '.' + p);
    if (!resolved.startsWith(distDir)) {
      res.writeHead(403).end('forbidden');
      return;
    }
    send(res, resolved);
  };
}
