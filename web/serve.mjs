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
//
// WebXR only runs on an HTTPS page (or on localhost), so a headset on the
// LAN needs TLS: set TLS_CERTIFICATE_FILE and TLS_KEY_FILE (the same
// variables dora-openarm-webxr uses) to serve HTTPS instead.
import http from "node:http";
import https from "node:https";
import os from "node:os";
import path from "node:path";

const ROOT = path.resolve(import.meta.dirname, "..");
const PORT = Number(process.argv[2] ?? process.env.PORT ?? 8080);
const TLS_CERTIFICATE_FILE = process.env.TLS_CERTIFICATE_FILE;
const TLS_KEY_FILE = process.env.TLS_KEY_FILE;

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

const handler = async (req, res) => {
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
};

let server;
let scheme = "http";
if (TLS_CERTIFICATE_FILE || TLS_KEY_FILE) {
  if (!TLS_CERTIFICATE_FILE || !TLS_KEY_FILE) {
    console.error("TLS_CERTIFICATE_FILE and TLS_KEY_FILE must be set together");
    process.exit(1);
  }
  server = https.createServer(
    {
      cert: await fs.readFile(TLS_CERTIFICATE_FILE),
      key: await fs.readFile(TLS_KEY_FILE),
    },
    handler,
  );
  scheme = "https";
} else {
  server = http.createServer(handler);
}

server.listen(PORT, () => {
  console.log(`Serving ${ROOT} at ${scheme}://localhost:${PORT}/`);
  if (scheme === "https") {
    console.log(`  from another device: ${scheme}://${os.hostname()}:${PORT}/`);
  }
});
