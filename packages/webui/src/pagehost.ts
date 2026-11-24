import * as THREE from 'three';
import { Host, type RpcRequest } from '@lightphyx/engine';

/**
 * Pool-member mode: this page hosts multiple headless envs and executes
 * protocol requests arriving over the /page WebSocket. Requests START strictly
 * in arrival order (material swaps and render targets are shared state), but a
 * request may release the queue early once it is only waiting on the GPU —
 * so E env renders pipeline their readbacks instead of idling one by one.
 */
export async function runPageHost(pageId: string): Promise<void> {
  const canvas = document.getElementById('app') as HTMLCanvasElement;
  const preview = new URLSearchParams(location.search).get('preview') === '1';
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: false });
  renderer.setSize(canvas.clientWidth || 640, canvas.clientHeight || 480);
  const host = new Host(renderer);

  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  const ws = new WebSocket(`${proto}://${location.host}/page?id=${encodeURIComponent(pageId)}`);
  ws.binaryType = 'arraybuffer';

  const sendFrame = (frame: Uint8Array<ArrayBuffer>) => ws.send(frame);

  let queue: Promise<void> = Promise.resolve();
  ws.onmessage = (ev) => {
    const req = JSON.parse(String(ev.data)) as RpcRequest;
    queue = queue.then(
      () =>
        new Promise<void>((next) => {
          let released = false;
          const release = () => {
            if (released) return;
            released = true;
            next();
          };
          host
            .handle(req, sendFrame, release)
            .then((result) => ws.send(JSON.stringify({ id: req.id, result: result ?? null })))
            .catch((err: unknown) => {
              const message = err instanceof Error ? err.message : String(err);
              ws.send(JSON.stringify({ id: req.id, error: { message } }));
            })
            .finally(() => {
              if (preview) host.renderPreview();
              release();
            });
        }),
    );
  };
  ws.onopen = () => ws.send(JSON.stringify({ hello: 'page', pageId }));
  ws.onclose = () => host.dispose();
}
