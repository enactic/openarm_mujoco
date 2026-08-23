// Copyright 2026 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

import fs from "node:fs/promises";
// Minimal static file server for OpenArm MuJoCo Web — a node:http stand-in
// for `python3 -m http.server` with no dependencies. Serves the repository
// root (one level above web/) so the top-level index.html, web/*.js, and the
// v2/ model files are all reachable. Run with: node serve.mjs [port]
import http from "node:http";
import path from "node:path";

const ROOT = path.resolve(import.meta.dirname, "..");
const PORT = Number(process.argv[2] ?? process.env.PORT ?? 8080);

// Module scripts are MIME-checked by browsers, so .js/.mjs must be
// text/javascript. Everything else here is served for completeness; fetch()
// of model assets (.xml, .stl, ...) does not care about the type.
const MIME = {
  ".html": "text/html; charset=utf-8",
  ".js": "text/javascript; charset=utf-8",
  ".mjs": "text/javascript; charset=utf-8",
  ".css": "text/css; charset=utf-8",
  ".json": "application/json",
  ".xml": "application/xml",
  ".wasm": "application/wasm",
  ".png": "image/png",
  ".jpg": "image/jpeg",
  ".svg": "image/svg+xml",
  ".ico": "image/x-icon",
  ".md": "text/markdown; charset=utf-8",
};

const server = http.createServer(async (req, res) => {
  const url = new URL(req.url, "http://localhost");
  let filePath = path.normalize(
    path.join(ROOT, decodeURIComponent(url.pathname)),
  );
  if (!filePath.startsWith(ROOT + path.sep) && filePath !== ROOT) {
    res.writeHead(403).end("Forbidden");
    return;
  }
  try {
    if ((await fs.stat(filePath)).isDirectory()) {
      filePath = path.join(filePath, "index.html");
    }
    const body = await fs.readFile(filePath);
    res.writeHead(200, {
      "content-type":
        MIME[path.extname(filePath).toLowerCase()] ??
        "application/octet-stream",
      "content-length": body.length,
    });
    res.end(body);
  } catch {
    res.writeHead(404).end("Not Found");
  }
});

server.listen(PORT, () => {
  console.log(`Serving ${ROOT} at http://localhost:${PORT}/`);
});
