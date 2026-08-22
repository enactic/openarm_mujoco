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

// Load a scene and everything it references into an MjVFS, mirroring the v2
// directory layout: sub-models (file="....xml") recurse, mesh assets resolve
// against each XML's own meshdir. readFile abstracts the byte source and
// DOMParserImpl the XML parser (fetch + the native DOMParser in the browser,
// fs + @xmldom/xmldom in the Node tests), so both sides load models by
// exactly the same rules.
export async function buildVFS(mujoco, scenePath, readFile, DOMParserImpl) {
  const vfs = new mujoco.MjVFS();
  const seen = new Set();
  const assets = [];
  const norm = (p) => {
    const out = [];
    for (const part of p.split("/")) {
      if (part === "" || part === ".") continue;
      if (part === "..") out.pop();
      else out.push(part);
    }
    return out.join("/");
  };
  const addXML = async (path) => {
    if (seen.has(path)) return;
    seen.add(path);
    const bytes = await readFile(path);
    const text = new TextDecoder().decode(bytes);
    vfs.addBuffer(path, bytes);
    const dir = path.includes("/")
      ? path.slice(0, path.lastIndexOf("/") + 1)
      : "";
    const doc = new DOMParserImpl().parseFromString(text, "text/xml");
    const elements = doc.getElementsByTagName("*");
    let meshdir = "";
    const refs = [];
    for (let i = 0; i < elements.length; i++) {
      meshdir ||= elements[i].getAttribute("meshdir") ?? "";
      const f = elements[i].getAttribute("file");
      if (f) refs.push([elements[i].tagName, f]);
    }
    for (const [tag, f] of refs) {
      if (tag === "model") {
        await addXML(norm(dir + f));
      } else if (tag === "mesh") {
        const p = norm(dir + (meshdir ? `${meshdir}/` : "") + f);
        if (!seen.has(p)) {
          seen.add(p);
          assets.push(p);
        }
      } else {
        // Other file-based assets (texture, hfield, skin, ...) resolve
        // against texturedir/assetdir, which this walker does not implement:
        // fail with the reason instead of silently registering the file
        // under a wrong, meshdir-relative VFS path.
        throw new Error(
          `${path}: unsupported file reference <${tag} file="${f}">`,
        );
      }
    }
  };
  try {
    await addXML(scenePath);
    await Promise.all(
      assets.map(async (p) => {
        vfs.addBuffer(p, await readFile(p));
      }),
    );
  } catch (e) {
    vfs.delete(); // a half-built VFS would otherwise leak in the WASM heap
    throw e;
  }
  return vfs;
}
