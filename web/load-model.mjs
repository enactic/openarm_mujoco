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

// Node-side wrapper for the shared VFS walker (model-vfs.js): load a v2
// scene from the repository checkout via fs, by the same rules the browser
// app uses.
import fs from "node:fs";
import path from "node:path";
import { DOMParser } from "@xmldom/xmldom";
import { buildVFS } from "./model-vfs.js";

const V2_DIR = path.resolve(import.meta.dirname, "../v2");

export async function loadSceneModel(mujoco, scenePath) {
  const vfs = await buildVFS(
    mujoco,
    scenePath,
    async (p) => new Uint8Array(fs.readFileSync(path.join(V2_DIR, p))),
    DOMParser,
  );
  try {
    return mujoco.MjModel.from_xml_path(scenePath, vfs);
  } finally {
    vfs.delete();
  }
}

export function loadCellModel(mujoco) {
  return loadSceneModel(mujoco, "cell/cell.xml");
}
