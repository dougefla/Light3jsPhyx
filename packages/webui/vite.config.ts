import { defineConfig, searchForWorkspaceRoot } from 'vite';
import os from 'node:os';

export default defineConfig({
  resolve: {
    // engine and webui must share a single three instance
    dedupe: ['three'],
  },
  server: {
    fs: {
      // allow /@fs/ access to datasets anywhere under the user's home in dev
      allow: [searchForWorkspaceRoot(process.cwd()), os.homedir()],
    },
  },
});
