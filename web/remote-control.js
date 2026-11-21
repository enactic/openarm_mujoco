// Copyright 2026 Enactic, Inc.
// SPDX-License-Identifier: Apache-2.0

// External joint control over WebSocket (ROS, Dora, or another adapter).
// Uses the cmd/state protocol shared with openarm_mujoco_hardware.
// The server identifies the joints and whether cmd values are positions or
// efforts. Never guess: legacy torque values must not become position targets.
export class RemoteControlBridge {
  constructor(mujoco, model, data, onClose, scenePath = "") {
    this.mujoco = mujoco;
    this.model = model;
    this.data = data;
    this.onClose = onClose;
    this.scenePath = scenePath;
    this.sessions = [];
    this.closed = false;
    this.status = "Connecting";
  }

  connect(urls) {
    try {
      for (const url of urls) {
        const socket = new WebSocket(url);
        const session = { socket, bindings: [], mode: null, lastCommand: null };
        this.sessions.push(session);
        socket.onopen = () => socket.send(JSON.stringify({ hello: true }));
        socket.onmessage = (event) => {
          try {
            this.receive(session, JSON.parse(event.data));
          } catch (error) {
            this.close(error.message);
          }
        };
        socket.onclose = () =>
          this.close("Disconnected; local control restored");
        socket.onerror = () => this.close(`Connection failed: ${url}`);
      }
    } catch (error) {
      this.close(error.message);
    }
  }

  bind(session, message) {
    if (session.mode) throw new Error("Repeated hardware handshake");
    if (
      message.scene_group &&
      this.scenePath.split("/")[0] !== message.scene_group
    )
      throw new Error(
        `Select a ${message.scene_group} scene before connecting this controller`,
      );
    if (!["position", "effort"].includes(message.command_interface))
      throw new Error("Unsupported hardware command interface");
    if (!Array.isArray(message.joints) || !message.joints.length)
      throw new Error("Missing hardware joint list");
    const { model, data, mujoco } = this;
    const claimed = new Set(
      this.sessions.flatMap((s) => s.bindings.map((b) => b.name)),
    );
    const bindings = message.joints.map((name) => {
      if (claimed.has(name)) throw new Error(`Joint claimed twice: ${name}`);
      claimed.add(name);
      const jid = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_JOINT.value,
        name,
      );
      if (jid < 0) throw new Error(`Unknown joint: ${name}`);
      if (
        ![
          mujoco.mjtJoint.mjJNT_HINGE.value,
          mujoco.mjtJoint.mjJNT_SLIDE.value,
        ].includes(model.jnt_type[jid])
      )
        throw new Error(`Only hinge/slide joints are supported: ${name}`);
      const actuators = [];
      for (let a = 0; a < model.nu; a++) {
        if (
          model.actuator_trntype[a] === mujoco.mjtTrn.mjTRN_JOINT.value &&
          model.actuator_trnid[2 * a] === jid
        )
          actuators.push(a);
      }
      if (actuators.length !== 1)
        throw new Error(`Expected one joint actuator: ${name}`);
      const aid = actuators[0];
      if (
        model.actuator_gear[6 * aid] !== 1 ||
        model.actuator_dyntype[aid] !== 0
      )
        throw new Error(`Expected a direct, stateless actuator: ${name}`);
      return {
        name,
        aid,
        qid: model.jnt_qposadr[jid],
        did: model.jnt_dofadr[jid],
        gain: [...model.actuator_gainprm.slice(10 * aid, 10 * aid + 10)],
        bias: [...model.actuator_biasprm.slice(10 * aid, 10 * aid + 10)],
        gainType: model.actuator_gaintype[aid],
        biasType: model.actuator_biastype[aid],
        limited: model.actuator_ctrllimited[aid],
      };
    });
    session.mode = message.command_interface;
    session.bindings = bindings;
    // Keep the authored position hold until the external controller sends commands.
    for (const b of bindings) data.ctrl[b.aid] = data.qpos[b.qid];
    this.status = this.sessions.every((s) => s.mode)
      ? `Connected: ${this.sessions.map((s) => `${s.mode} (${s.bindings.length} joints)`).join(", ")}`
      : "Waiting for all hardware interfaces";
    this.publish(session);
  }

  receive(session, message) {
    if (message.joints) {
      this.bind(session, message);
      return;
    }
    if (!session.mode) throw new Error("Hardware handshake required");
    if (message.enabled === false) {
      this.close("External controller stopped; local control restored");
      return;
    }
    if (message.command_interface !== session.mode || !message.cmd)
      throw new Error("Unexpected hardware command");
    // Validate a complete command before changing any actuator.
    const values = session.bindings.map((b) => message.cmd[b.name]);
    if (!values.every((v) => typeof v === "number" && Number.isFinite(v)))
      throw new Error("Incomplete or non-finite hardware command");
    if (session.mode === "effort" && session.lastCommand === null) {
      const { model, mujoco } = this;
      for (const b of session.bindings) {
        // Runtime conversion only. Keep authored actuator force limits.
        model.actuator_gaintype[b.aid] = mujoco.mjtGain.mjGAIN_FIXED.value;
        model.actuator_biastype[b.aid] = mujoco.mjtBias.mjBIAS_NONE.value;
        model.actuator_gainprm.fill(0, 10 * b.aid, 10 * b.aid + 10);
        model.actuator_gainprm[10 * b.aid] = 1;
        model.actuator_biasprm.fill(0, 10 * b.aid, 10 * b.aid + 10);
        model.actuator_ctrllimited[b.aid] = 0;
      }
    }
    values.forEach((v, i) => {
      this.data.ctrl[session.bindings[i].aid] = v;
    });
    session.lastCommand = performance.now();
  }

  beforeStep() {
    for (const session of this.sessions) {
      if (
        session.lastCommand !== null &&
        performance.now() - session.lastCommand > 500
      ) {
        this.close("External commands timed out; local control restored");
        return;
      }
      // The external controller owns torque control. Do not add unrestricted Web bias forces.
      if (session.mode === "effort" && session.lastCommand !== null) {
        for (const b of session.bindings) this.data.qfrc_applied[b.did] = 0;
      }
    }
  }

  publish(session) {
    if (!session.mode || session.socket.readyState !== WebSocket.OPEN) return;
    if (session.socket.bufferedAmount > 0) return; // feedback must not accumulate
    const state = {};
    for (const b of session.bindings) {
      state[b.name] = {
        qpos: this.data.qpos[b.qid],
        qvel: this.data.qvel[b.did],
        // Actuator effort only: excludes contact forces and Web bias compensation.
        qtau: this.data.qfrc_actuator[b.did],
      };
    }
    session.socket.send(JSON.stringify({ state }));
  }

  afterStep() {
    for (const session of this.sessions) this.publish(session);
  }

  close(reason = "Disconnected; local control restored") {
    if (this.closed) return;
    this.closed = true;
    for (const session of this.sessions) {
      session.socket.onopen = null;
      session.socket.onclose = null;
      session.socket.onerror = null;
      session.socket.onmessage = null;
      session.socket.close();
      for (const b of session.bindings) {
        this.model.actuator_gainprm.set(b.gain, 10 * b.aid);
        this.model.actuator_biasprm.set(b.bias, 10 * b.aid);
        this.model.actuator_gaintype[b.aid] = b.gainType;
        this.model.actuator_biastype[b.aid] = b.biasType;
        this.model.actuator_ctrllimited[b.aid] = b.limited;
        this.data.ctrl[b.aid] = this.data.qpos[b.qid];
      }
    }
    this.onClose(reason);
  }
}
