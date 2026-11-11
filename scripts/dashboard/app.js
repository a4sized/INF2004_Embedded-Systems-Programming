(() => {
  "use strict";

  const SVG_NS = "http://www.w3.org/2000/svg";
  const STALE_THRESHOLD_SEC = 3.5;

  const STATE_INFO = {
    0: { tone: "idle", desc: "Motors disarmed / stationary." },
    1: { tone: "info", desc: "Smooth PWM ramp-up toward cruise speed." },
    2: { tone: "ok", desc: "Nominal autonomous forward drive." },
    3: { tone: "warn", desc: "Active reversing maneuver (obstacle / recovery)." },
    4: { tone: "alert", desc: "Safety halt latched (E-STOP, obstacle, or fault)." },
    5: { tone: "info", desc: "Descending ramp / hump from S4 IMU estimation." },
    6: { tone: "alert", desc: "Abnormal lift / pickup detected by S4 IMU." },
    7: { tone: "alert", desc: "Vehicle rollover / extreme tilt fault detected." },
  };

  const ui = {
    connPill: document.getElementById("conn-pill"),
    connLabel: document.getElementById("conn-label"),
    topicTelem: document.getElementById("topic-telem"),
    topicCmd: document.getElementById("topic-cmd"),
    brokerForm: document.getElementById("broker-form"),
    brokerIp: document.getElementById("broker-ip"),
    brokerPort: document.getElementById("broker-port"),
    demoBtn: document.getElementById("demo-btn"),
    exportTxtBtn: document.getElementById("export-txt-btn"),
    exportCsvBtn: document.getElementById("export-csv-btn"),
    resetBtn: document.getElementById("reset-btn"),
    errorBanner: document.getElementById("error-banner"),

    kpiSeq: document.getElementById("kpi-seq"),
    kpiRawHex: document.getElementById("kpi-raw-hex"),
    kpiRx: document.getElementById("kpi-rx"),
    kpiInvalid: document.getElementById("kpi-invalid"),
    kpiAge: document.getElementById("kpi-age"),
    kpiRxTime: document.getElementById("kpi-rx-time"),
    kpiDrops: document.getElementById("kpi-drops"),
    kpiMaxHump: document.getElementById("kpi-max-hump"),

    stateIdBadge: document.getElementById("state-id-badge"),
    stateHero: document.getElementById("state-hero"),
    stateName: document.getElementById("state-name"),
    stateDesc: document.getElementById("state-desc"),
    stateHistory: document.getElementById("state-history"),

    faultHexBadge: document.getElementById("fault-hex-badge"),
    faultSummary: document.getElementById("fault-summary"),
    faultGrid: document.getElementById("fault-grid"),

    humpBadge: document.getElementById("hump-badge"),
    carTiltGroup: document.getElementById("car-tilt-group"),
    pitchHorizonText: document.getElementById("pitch-horizon-text"),
    pitchValue: document.getElementById("pitch-value"),
    humpHeightValue: document.getElementById("hump-height-value"),
    humpStateValue: document.getElementById("hump-state-value"),

    lineBinBadge: document.getElementById("line-bin-badge"),
    obstacleValue: document.getElementById("obstacle-value"),
    obstacleBar: document.getElementById("obstacle-bar"),
    lineHexVal: document.getElementById("line-hex-val"),
    irLeds: Array.from(document.querySelectorAll("#ir-leds .ir-bit")),
    barcodeRawId: document.getElementById("barcode-raw-id"),
    barcodeGlyph: document.getElementById("barcode-glyph"),
    barcodeHistory: document.getElementById("barcode-history"),

    chartPitchRange: document.getElementById("chart-pitch-range"),
    chartPitch: document.getElementById("chart-pitch"),
    chartObsLatest: document.getElementById("chart-obs-latest"),
    chartObstacle: document.getElementById("chart-obstacle"),
    chartHumpLatest: document.getElementById("chart-hump-latest"),
    chartHump: document.getElementById("chart-hump"),

    modeButtons: Array.from(document.querySelectorAll("[data-cmd-mode]")),
    cmdButtons: Array.from(document.querySelectorAll("[data-cmd]")),
    customCmdForm: document.getElementById("custom-cmd-form"),
    customCmdInput: document.getElementById("custom-cmd-input"),
    cmdFeedback: document.getElementById("cmd-feedback"),
    cmdLog: document.getElementById("cmd-log"),

    tabButtons: Array.from(document.querySelectorAll("[data-tab]")),
    filterFaults: document.getElementById("filter-faults"),
    pauseScroll: document.getElementById("pause-scroll"),
    terminalOutput: document.getElementById("terminal-output"),
    pktTableBody: document.getElementById("pkt-table-body"),
    tableWrap: document.querySelector("#tab-table .table-wrap"),
    eventList: document.getElementById("event-list"),
  };

  const store = {
    meta: null,
    faultDefs: [],
    packets: [],
    events: [],
    commands: [],
    barcodes: [],
    stateTransitions: [],
    cmdMode: "ascii",
    sseConnected: true,
    lastRxMs: null,
  };

  function pad3(num) {
    return String(num).padStart(3, "0");
  }

  function formatSignedDeg(deg, width) {
    const sign = deg >= 0 ? "+" : "-";
    const absStr = String(Math.abs(deg));
    const str = `${sign}${absStr}`;
    return width ? str.padStart(width, " ") : str;
  }

  function formatDecoderBlock(pkt, includeTime) {
    const barcodeText = pkt.barcode_char
      ? pkt.barcode_char
      : String(pkt.barcode_id);
    const pitchText = formatSignedDeg(pkt.pitch_deg, 3);
    const obsText = String(pkt.obstacle_cm).padStart(3, " ");
    const timeSuffix = includeTime && pkt.rx_time ? `  @ ${pkt.rx_time}` : "";
    return [
      `[SEQ #${pad3(pkt.seq)}] Raw HEX: [${pkt.raw_hex}] (8 bytes)${timeSuffix}`,
      `  ├─ State:     ${pkt.car_state_name} (${pkt.car_state})`,
      `  ├─ Terrain:   ${pkt.hump_state_name} (${pkt.hump_state}) | Peak Hump Height = ${pkt.hump_height_mm} mm | Pitch = ${pitchText}°`,
      `  ├─ Sensors:   Obstacle = ${obsText} cm | Line IR = [${pkt.line_bin}] | Barcode = '${barcodeText}'`,
      `  └─ Faults:    [${pkt.fault_hex}] -> ${pkt.fault_summary}`,
      "-".repeat(80),
    ].join("\n");
  }

  function buildFaultGrid(faultDefs) {
    if (!faultDefs || faultDefs.length === 0) return;
    store.faultDefs = faultDefs;
    ui.faultGrid.replaceChildren();
    for (const f of faultDefs) {
      const tile = document.createElement("div");
      tile.className = "fault-tile";
      tile.dataset.bit = String(f.bit);
      tile.dataset.tier = f.tier;

      const top = document.createElement("div");
      top.className = "f-top";
      const hexSpan = document.createElement("span");
      hexSpan.className = "mono";
      hexSpan.textContent = f.hex;
      const tierSpan = document.createElement("span");
      tierSpan.textContent =
        f.tier === "tier1" ? "TIER 1" : f.tier === "tier2" ? "TIER 2" : "SYS/NET";
      top.append(hexSpan, tierSpan);

      const name = document.createElement("div");
      name.className = "f-name";
      name.textContent = f.name;

      tile.append(top, name);
      ui.faultGrid.appendChild(tile);
    }
  }

  function setErrorBanner(message) {
    if (message) {
      ui.errorBanner.textContent = message;
      ui.errorBanner.classList.remove("hidden");
    } else {
      ui.errorBanner.textContent = "";
      ui.errorBanner.classList.add("hidden");
    }
  }

  function renderMeta(meta, syncInputs) {
    if (!meta) return;
    store.meta = meta;

    ui.topicTelem.textContent = meta.topic_telem;
    ui.topicCmd.textContent = meta.topic_cmd;
    ui.kpiRx.textContent = String(meta.rx_count);
    ui.kpiInvalid.textContent = `${meta.invalid_count} invalid frame${meta.invalid_count === 1 ? "" : "s"}`;
    ui.kpiDrops.textContent = String(meta.seq_drops);
    ui.kpiMaxHump.textContent = `${meta.session_max_hump_mm} mm`;

    if (syncInputs) {
      if (document.activeElement !== ui.brokerIp) {
        ui.brokerIp.value = meta.broker_ip;
      }
      if (document.activeElement !== ui.brokerPort) {
        ui.brokerPort.value = String(meta.broker_port);
      }
    }

    const demoOn = Boolean(meta.demo_mode);
    ui.demoBtn.classList.toggle("active", demoOn);
    ui.demoBtn.setAttribute("aria-pressed", String(demoOn));
    ui.demoBtn.textContent = demoOn ? "Demo Sim: ON" : "Demo Sim: OFF";

    if (meta.last_rx_epoch) {
      const serverAgeMs = Math.max(0, Date.now() - meta.last_rx_epoch * 1000);
      if (store.lastRxMs === null || serverAgeMs < Date.now() - store.lastRxMs) {
        store.lastRxMs = meta.last_rx_epoch * 1000;
      }
    }

    if (!store.sseConnected) {
      setErrorBanner("Lost connection to local Python dashboard server — retrying SSE stream...");
    } else if (meta.mqtt_error && !demoOn) {
      setErrorBanner(meta.mqtt_error);
    } else {
      setErrorBanner("");
    }

    updateConnectionPill();
  }

  function updateConnectionPill() {
    const meta = store.meta;
    if (!store.sseConnected) {
      ui.connPill.dataset.status = "error";
      ui.connLabel.textContent = "SERVER OFFLINE";
      return;
    }
    if (!meta) return;

    if (meta.demo_mode) {
      ui.connPill.dataset.status = "demo";
      ui.connLabel.textContent = "DEMO SIM";
      return;
    }

    const status = meta.mqtt_status || "disconnected";
    if (status === "connected" && store.lastRxMs !== null) {
      const ageSec = (Date.now() - store.lastRxMs) / 1000;
      if (ageSec > STALE_THRESHOLD_SEC) {
        ui.connPill.dataset.status = "stale";
        ui.connLabel.textContent = `STALE (${ageSec.toFixed(0)}s)`;
        return;
      }
    }

    const labels = {
      connected: "LIVE MQTT",
      connecting: "CONNECTING",
      disconnected: "DISCONNECTED",
      error: "MQTT ERROR",
      demo: "DEMO SIM",
    };
    ui.connPill.dataset.status = status;
    ui.connLabel.textContent = labels[status] || status.toUpperCase();
  }

  function renderLatestPacket(pkt) {
    if (!pkt) {
      ui.kpiSeq.textContent = "#---";
      ui.kpiRawHex.textContent = "[-- -- -- -- -- -- -- --]";
      ui.kpiAge.textContent = "--";
      ui.kpiRxTime.textContent = "waiting for packet";
      ui.stateIdBadge.textContent = "ID: -";
      ui.stateHero.dataset.tone = "idle";
      ui.stateName.textContent = "WAITING FOR TELEMETRY";
      ui.stateDesc.textContent = "No 8-byte binary frame received yet.";
      ui.faultHexBadge.textContent = "0x000";
      ui.faultSummary.dataset.tone = "ok";
      ui.faultSummary.textContent = "NONE (ALL CLEAR)";
      for (const tile of ui.faultGrid.children) {
        tile.classList.remove("active-tier1", "active-tier2", "active-info");
      }
      ui.humpBadge.textContent = "FLAT (0)";
      ui.carTiltGroup.setAttribute("transform", "translate(110 72) rotate(0)");
      ui.pitchHorizonText.textContent = "+0° (NOSE LEVEL)";
      ui.pitchValue.textContent = "+0°";
      ui.humpHeightValue.textContent = "0 mm";
      ui.humpStateValue.textContent = "FLAT (0)";
      ui.lineBinBadge.textContent = "0b00000";
      ui.obstacleValue.textContent = "511 cm (CLEAR)";
      ui.obstacleBar.style.width = "0%";
      ui.obstacleBar.className = "bar-fill";
      ui.lineHexVal.textContent = "0x00";
      for (const bitEl of ui.irLeds) bitEl.classList.remove("on");
      ui.barcodeRawId.textContent = "id=0";
      ui.barcodeGlyph.textContent = "NONE";
      ui.barcodeGlyph.classList.remove("active");
      return;
    }

    ui.kpiSeq.textContent = `#${pad3(pkt.seq)}`;
    ui.kpiRawHex.textContent = `[${pkt.raw_hex}]`;
    ui.kpiRxTime.textContent = `${pkt.rx_time} (${pkt.source.toUpperCase()})`;

    const info = STATE_INFO[pkt.car_state] || { tone: "info", desc: "Robot state update." };
    ui.stateIdBadge.textContent = `ID: ${pkt.car_state}`;
    ui.stateHero.dataset.tone = info.tone;
    ui.stateName.textContent = pkt.car_state_name;
    ui.stateDesc.textContent = info.desc;

    ui.faultHexBadge.textContent = pkt.fault_hex;
    ui.faultSummary.textContent = pkt.fault_summary;
    ui.faultSummary.dataset.tone = pkt.fault_mask === 0
      ? "ok"
      : pkt.tier1_fault
      ? "alert"
      : "warn";

    for (const tile of ui.faultGrid.children) {
      const bit = Number(tile.dataset.bit || "0");
      const tier = tile.dataset.tier || "info";
      tile.classList.remove("active-tier1", "active-tier2", "active-info");
      if ((pkt.fault_mask & bit) !== 0) {
        tile.classList.add(`active-${tier}`);
      }
    }

    const humpText = `${pkt.hump_state_name} (${pkt.hump_state})`;
    ui.humpBadge.textContent = humpText;
    const clampedPitch = Math.max(-45, Math.min(45, pkt.pitch_deg));
    ui.carTiltGroup.setAttribute("transform", `translate(110 72) rotate(${-clampedPitch})`);
    const pitchDir = pkt.pitch_deg > 2 ? "NOSE UP" : pkt.pitch_deg < -2 ? "NOSE DOWN" : "NOSE LEVEL";
    const pitchFormatted = `${formatSignedDeg(pkt.pitch_deg, 0)}°`;
    ui.pitchHorizonText.textContent = `${pitchFormatted} (${pitchDir})`;
    ui.pitchValue.textContent = pitchFormatted;
    ui.humpHeightValue.textContent = `${pkt.hump_height_mm} mm`;
    ui.humpStateValue.textContent = humpText;

    if (pkt.obstacle_clear) {
      ui.obstacleValue.textContent = "511 cm (CLEAR)";
      ui.obstacleBar.style.width = "100%";
      ui.obstacleBar.className = "bar-fill";
    } else {
      const zone = pkt.obstacle_cm <= 5 ? " (STOP)" : pkt.obstacle_cm <= 35 ? " (DETECT)" : "";
      ui.obstacleValue.textContent = `${pkt.obstacle_cm} cm${zone}`;
      const capped = Math.max(0, Math.min(50, pkt.obstacle_cm));
      const pct = Math.max(4, (capped / 50) * 100);
      ui.obstacleBar.style.width = `${pct.toFixed(1)}%`;
      ui.obstacleBar.className =
        pkt.obstacle_cm <= 10 ? "bar-fill stop" : pkt.obstacle_cm <= 35 ? "bar-fill near" : "bar-fill";
    }

    ui.lineBinBadge.textContent = pkt.line_bin;
    ui.lineHexVal.textContent = `0x${pkt.line_mask.toString(16).toUpperCase().padStart(2, "0")}`;
    const bits = pkt.line_bits || [0, 0, 0, 0, 0];
    ui.irLeds.forEach((el, idx) => {
      el.classList.toggle("on", Boolean(bits[idx]));
    });

    ui.barcodeRawId.textContent = `id=${pkt.barcode_id}`;
    ui.barcodeGlyph.textContent = pkt.barcode_display;
    ui.barcodeGlyph.classList.toggle("active", pkt.barcode_id !== 0);
  }

  function renderStateTransitions() {
    ui.stateHistory.replaceChildren();
    if (store.stateTransitions.length === 0) {
      const empty = document.createElement("span");
      empty.className = "chip muted";
      empty.textContent = "None yet";
      ui.stateHistory.appendChild(empty);
      return;
    }
    const recent = store.stateTransitions.slice(-6).reverse();
    for (const item of recent) {
      const chip = document.createElement("span");
      chip.className = "chip";
      chip.textContent = `${item.time} ${item.name} (${item.state})`;
      ui.stateHistory.appendChild(chip);
    }
  }

  function renderBarcodes() {
    ui.barcodeHistory.replaceChildren();
    if (!store.barcodes || store.barcodes.length === 0) {
      const empty = document.createElement("span");
      empty.className = "chip muted";
      empty.textContent = "No scans";
      ui.barcodeHistory.appendChild(empty);
      return;
    }
    for (const b of store.barcodes.slice(-6).reverse()) {
      const chip = document.createElement("span");
      chip.className = "chip";
      chip.textContent = `'${b.char}' #${pad3(b.seq)}`;
      ui.barcodeHistory.appendChild(chip);
    }
  }

  function svgEl(tag, attrs) {
    const el = document.createElementNS(SVG_NS, tag);
    for (const [k, v] of Object.entries(attrs)) {
      el.setAttribute(k, String(v));
    }
    return el;
  }

  function drawLineChart(svg, values, opts) {
    const W = 420;
    const H = 120;
    const padL = 34;
    const padR = 10;
    const padT = 10;
    const padB = 18;
    const innerW = W - padL - padR;
    const innerH = H - padT - padB;

    svg.replaceChildren();

    const yMin = opts.yMin;
    const yMax = opts.yMax;
    const span = Math.max(1, yMax - yMin);
    const toY = (v) => padT + innerH - ((Math.max(yMin, Math.min(yMax, v)) - yMin) / span) * innerH;

    // Reference grid lines
    const refs = opts.refs || [yMin, yMax];
    for (const r of refs) {
      const y = toY(r.value !== undefined ? r.value : r);
      const val = r.value !== undefined ? r.value : r;
      svg.appendChild(
        svgEl("line", {
          x1: padL,
          y1: y.toFixed(1),
          x2: W - padR,
          y2: y.toFixed(1),
          stroke: r.color || "#1e2c3f",
          "stroke-width": r.width || 1,
          "stroke-dasharray": r.dash || "3 3",
        })
      );
      const lbl = svgEl("text", {
        x: padL - 4,
        y: (y + 3).toFixed(1),
        "text-anchor": "end",
        fill: r.color || "#6f849e",
        "font-size": "9",
        "font-family": "var(--mono)",
      });
      lbl.textContent = String(val);
      svg.appendChild(lbl);
    }

    if (!values || values.length === 0) {
      const txt = svgEl("text", {
        x: W / 2,
        y: H / 2 + 3,
        "text-anchor": "middle",
        fill: "#64748b",
        "font-size": "11",
        "font-family": "var(--mono)",
      });
      txt.textContent = "No packets yet";
      svg.appendChild(txt);
      return;
    }

    const pts = values.map((v, i) => {
      const x =
        values.length === 1
          ? padL + innerW / 2
          : padL + (i / (values.length - 1)) * innerW;
      const y = toY(v);
      return `${x.toFixed(1)},${y.toFixed(1)}`;
    });

    if (opts.fillArea && values.length > 1) {
      const baseY = toY(yMin).toFixed(1);
      const polyPts = [`${padL},${baseY}`, ...pts, `${(padL + innerW).toFixed(1)},${baseY}`].join(" ");
      svg.appendChild(
        svgEl("polygon", {
          points: polyPts,
          fill: opts.fillArea,
        })
      );
    }

    if (values.length > 1) {
      svg.appendChild(
        svgEl("polyline", {
          points: pts.join(" "),
          fill: "none",
          stroke: opts.stroke || "#38bdf8",
          "stroke-width": 2,
          "stroke-linejoin": "round",
          "stroke-linecap": "round",
        })
      );
    }

    const lastPt = pts[pts.length - 1].split(",");
    svg.appendChild(
      svgEl("circle", {
        cx: lastPt[0],
        cy: lastPt[1],
        r: 3.5,
        fill: opts.stroke || "#38bdf8",
        stroke: "#0a1018",
        "stroke-width": 1.5,
      })
    );
  }

  function renderCharts() {
    const windowPkts = store.packets.slice(-90);
    const pitches = windowPkts.map((p) => p.pitch_deg);
    const maxAbsPitch = Math.max(20, ...pitches.map((v) => Math.abs(v)));
    const pBound = Math.ceil(maxAbsPitch / 5) * 5;
    ui.chartPitchRange.textContent = `-${pBound}..+${pBound}°`;
    drawLineChart(ui.chartPitch, pitches, {
      yMin: -pBound,
      yMax: pBound,
      stroke: "#38bdf8",
      refs: [
        { value: pBound },
        { value: 0, color: "#475569" },
        { value: -pBound },
      ],
    });

    const obsCapped = windowPkts.map((p) => Math.min(60, p.obstacle_cm));
    const latestPkt = windowPkts[windowPkts.length - 1];
    ui.chartObsLatest.textContent = latestPkt
      ? latestPkt.obstacle_clear
        ? "CLEAR (511)"
        : `${latestPkt.obstacle_cm} cm`
      : "--";
    drawLineChart(ui.chartObstacle, obsCapped, {
      yMin: 0,
      yMax: 60,
      stroke: "#f59e0b",
      refs: [
        { value: 60 },
        { value: 35, color: "#f59e0b" },
        { value: 5, color: "#ef4444" },
      ],
    });

    const humps = windowPkts.map((p) => p.hump_height_mm);
    const humpTop = Math.max(50, ...humps, 0);
    const humpCeiling = Math.ceil(humpTop / 10) * 10;
    ui.chartHumpLatest.textContent = latestPkt ? `${latestPkt.hump_height_mm} mm` : "--";
    drawLineChart(ui.chartHump, humps, {
      yMin: 0,
      yMax: humpCeiling,
      stroke: "#22c55e",
      fillArea: "rgba(34, 197, 94, 0.14)",
      refs: [{ value: humpCeiling }, { value: Math.round(humpCeiling / 2) }, { value: 0 }],
    });
  }

  function renderLogs() {
    const faultsOnly = ui.filterFaults.checked;
    const filtered = faultsOnly
      ? store.packets.filter((p) => p.fault_mask !== 0)
      : store.packets;

    if (filtered.length === 0) {
      ui.terminalOutput.textContent = faultsOnly
        ? "No packets with active faults recorded yet."
        : "Waiting for 8-byte packets on robot/inf2004team15/telemetry...";
    } else {
      const blocks = filtered.slice(-80).map((p) => formatDecoderBlock(p, true));
      ui.terminalOutput.textContent = blocks.join("\n");
      if (!ui.pauseScroll.checked) {
        ui.terminalOutput.scrollTop = ui.terminalOutput.scrollHeight;
      }
    }

    // Packet Table (newest first for quick inspection)
    ui.pktTableBody.replaceChildren();
    const rows = filtered.slice(-80).reverse();
    for (const pkt of rows) {
      const tr = document.createElement("tr");
      if (pkt.tier1_fault) tr.className = "row-alert";
      else if (pkt.fault_mask !== 0 || pkt.tier2_fault) tr.className = "row-warn";

      const cells = [
        pkt.rx_time || "",
        `#${pad3(pkt.seq)}`,
        pkt.raw_hex,
        `${pkt.car_state_name} (${pkt.car_state})`,
        `${pkt.hump_state_name} (${pkt.hump_state})`,
        `${pkt.hump_height_mm} mm`,
        `${formatSignedDeg(pkt.pitch_deg, 0)}°`,
        pkt.obstacle_clear ? "511 (CLR)" : `${pkt.obstacle_cm} cm`,
        pkt.line_bin,
        pkt.barcode_display,
        pkt.fault_mask === 0 ? "0x000 CLEAR" : `${pkt.fault_hex} ${pkt.fault_summary}`,
      ];
      for (const val of cells) {
        const td = document.createElement("td");
        td.textContent = val;
        tr.appendChild(td);
      }
      ui.pktTableBody.appendChild(tr);
    }
  }

  function renderEvents() {
    ui.eventList.replaceChildren();
    for (const ev of store.events.slice(-80)) {
      const row = document.createElement("div");
      row.className = `ev-row ${ev.level || "info"}`;
      const t = document.createElement("span");
      t.className = "muted";
      t.textContent = ev.time || "";
      const k = document.createElement("strong");
      k.textContent = `[${ev.kind}]`;
      const m = document.createElement("span");
      m.textContent = ev.message;
      row.append(t, k, m);
      ui.eventList.appendChild(row);
    }
    if (!ui.pauseScroll.checked) {
      ui.eventList.scrollTop = ui.eventList.scrollHeight;
    }
  }

  function renderCommands() {
    ui.cmdLog.replaceChildren();
    if (!store.commands || store.commands.length === 0) {
      const empty = document.createElement("div");
      empty.textContent = "No commands published yet.";
      ui.cmdLog.appendChild(empty);
      return;
    }
    for (const cmd of store.commands.slice().reverse()) {
      const line = document.createElement("div");
      line.textContent = `[${cmd.time}] ${cmd.display} | ${cmd.hex} | ${cmd.status}`;
      ui.cmdLog.appendChild(line);
    }
  }

  function rebuildStateTransitionsFromHistory(history) {
    store.stateTransitions = [];
    let prev = null;
    for (const pkt of history) {
      if (prev === null || pkt.car_state !== prev) {
        store.stateTransitions.push({
          time: pkt.rx_time || "",
          state: pkt.car_state,
          name: pkt.car_state_name,
        });
        prev = pkt.car_state;
      }
    }
  }

  function applyFullSnapshot(data) {
    if (data.dictionaries && data.dictionaries.faults) {
      buildFaultGrid(data.dictionaries.faults);
    }
    store.packets = Array.isArray(data.history) ? data.history.slice() : [];
    store.events = Array.isArray(data.events) ? data.events.slice() : [];
    store.commands = Array.isArray(data.commands) ? data.commands.slice() : [];
    store.barcodes = Array.isArray(data.barcodes) ? data.barcodes.slice() : [];
    rebuildStateTransitionsFromHistory(store.packets);

    if (data.latest && data.latest.rx_epoch) {
      store.lastRxMs = data.latest.rx_epoch * 1000;
    } else if (data.meta && data.meta.last_rx_epoch) {
      store.lastRxMs = data.meta.last_rx_epoch * 1000;
    } else {
      store.lastRxMs = null;
    }

    renderMeta(data.meta, true);
    renderLatestPacket(data.latest || store.packets[store.packets.length - 1] || null);
    renderStateTransitions();
    renderBarcodes();
    renderCharts();
    renderLogs();
    renderEvents();
    renderCommands();
  }

  function appendPacket(payload) {
    const pkt = payload.packet;
    if (!pkt) return;

    store.lastRxMs = pkt.rx_epoch ? pkt.rx_epoch * 1000 : Date.now();
    store.packets.push(pkt);
    if (store.packets.length > 240) {
      store.packets.splice(0, store.packets.length - 240);
    }

    const lastTr = store.stateTransitions[store.stateTransitions.length - 1];
    if (!lastTr || lastTr.state !== pkt.car_state) {
      store.stateTransitions.push({
        time: pkt.rx_time || "",
        state: pkt.car_state,
        name: pkt.car_state_name,
      });
      if (store.stateTransitions.length > 30) {
        store.stateTransitions.splice(0, store.stateTransitions.length - 30);
      }
    }

    if (Array.isArray(payload.events) && payload.events.length > 0) {
      store.events.push(...payload.events);
      if (store.events.length > 160) {
        store.events.splice(0, store.events.length - 160);
      }
    }
    if (Array.isArray(payload.barcodes)) {
      store.barcodes = payload.barcodes;
    }

    renderMeta(payload.meta, false);
    renderLatestPacket(pkt);
    renderStateTransitions();
    renderBarcodes();
    renderCharts();
    renderLogs();
    renderEvents();
  }

  async function postJson(url, body) {
    const res = await fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body || {}),
    });
    const data = await res.json().catch(() => ({}));
    if (!res.ok) {
      throw new Error(data.error || `HTTP ${res.status}`);
    }
    return data;
  }

  async function sendCommand(commandText) {
    try {
      ui.cmdFeedback.textContent = `Sending '${commandText}' (${store.cmdMode.toUpperCase()})...`;
      const res = await postJson("/api/cmd", {
        command: commandText,
        mode: store.cmdMode,
      });
      if (res.entry) {
        ui.cmdFeedback.textContent = `TX OK: ${res.entry.display} [${res.entry.hex}]`;
      }
    } catch (err) {
      ui.cmdFeedback.textContent = `TX Error: ${err.message}`;
    }
  }

  function downloadFile(filename, content, mime) {
    const blob = new Blob([content], { type: mime });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    a.remove();
    URL.revokeObjectURL(url);
  }

  function csvCell(val) {
    const s = String(val ?? "");
    return /[",\n]/.test(s) ? `"${s.replace(/"/g, '""')}"` : s;
  }

  function wireControls() {
    ui.brokerForm.addEventListener("submit", async (e) => {
      e.preventDefault();
      const ip = ui.brokerIp.value.trim();
      const port = Number(ui.brokerPort.value || "1883");
      try {
        const res = await postJson("/api/broker", { broker_ip: ip, broker_port: port });
        renderMeta(res.meta, true);
      } catch (err) {
        setErrorBanner(err.message);
      }
    });

    ui.demoBtn.addEventListener("click", async () => {
      const currentlyDemo = Boolean(store.meta && store.meta.demo_mode);
      try {
        const res = await postJson("/api/demo", { enabled: !currentlyDemo });
        renderMeta(res.meta, false);
      } catch (err) {
        setErrorBanner(err.message);
      }
    });

    ui.resetBtn.addEventListener("click", async () => {
      try {
        await postJson("/api/reset", {});
      } catch (err) {
        setErrorBanner(err.message);
      }
    });

    ui.exportTxtBtn.addEventListener("click", () => {
      if (store.packets.length === 0) return;
      const stamp = new Date().toISOString().replace(/[:.]/g, "-").slice(0, 19);
      const body = store.packets.map((p) => formatDecoderBlock(p, false)).join("\n") + "\n";
      downloadFile(`telemetry_log_${stamp}.txt`, body, "text/plain;charset=utf-8");
    });

    ui.exportCsvBtn.addEventListener("click", () => {
      if (store.packets.length === 0) return;
      const stamp = new Date().toISOString().replace(/[:.]/g, "-").slice(0, 19);
      const header = [
        "rx_time", "seq", "raw_hex", "car_state", "car_state_name",
        "hump_state", "hump_state_name", "hump_height_mm", "pitch_deg",
        "obstacle_cm", "line_mask", "line_bin", "barcode_id",
        "barcode_display", "fault_hex", "fault_summary",
      ].join(",");
      const lines = store.packets.map((p) =>
        [
          p.rx_time, p.seq, p.raw_hex, p.car_state, p.car_state_name,
          p.hump_state, p.hump_state_name, p.hump_height_mm, p.pitch_deg,
          p.obstacle_cm, p.line_mask, p.line_bin, p.barcode_id,
          p.barcode_display, p.fault_hex, p.fault_summary,
        ].map(csvCell).join(",")
      );
      downloadFile(`telemetry_log_${stamp}.csv`, [header, ...lines].join("\n") + "\n", "text/csv;charset=utf-8");
    });

    for (const btn of ui.modeButtons) {
      btn.addEventListener("click", () => {
        store.cmdMode = btn.dataset.cmdMode || "ascii";
        ui.modeButtons.forEach((b) => b.classList.toggle("active", b === btn));
      });
    }

    for (const btn of ui.cmdButtons) {
      btn.addEventListener("click", () => {
        const cmd = btn.dataset.cmd || "";
        if (cmd) sendCommand(cmd);
      });
    }

    ui.customCmdForm.addEventListener("submit", (e) => {
      e.preventDefault();
      const val = ui.customCmdInput.value.trim();
      if (!val) return;
      sendCommand(val);
      ui.customCmdInput.value = "";
    });

    for (const tabBtn of ui.tabButtons) {
      tabBtn.addEventListener("click", () => {
        const target = tabBtn.dataset.tab;
        ui.tabButtons.forEach((b) => b.classList.toggle("active", b === tabBtn));
        for (const pane of document.querySelectorAll(".tab-pane")) {
          pane.classList.toggle("active", pane.id === `tab-${target}`);
        }
      });
    }

    ui.filterFaults.addEventListener("change", renderLogs);
    ui.pauseScroll.addEventListener("change", () => {
      if (!ui.pauseScroll.checked) {
        ui.terminalOutput.scrollTop = ui.terminalOutput.scrollHeight;
        ui.eventList.scrollTop = ui.eventList.scrollHeight;
      }
    });
  }

  function startStream() {
    const es = new EventSource("/events");

    es.onopen = () => {
      store.sseConnected = true;
      renderMeta(store.meta, false);
    };

    es.onerror = () => {
      store.sseConnected = false;
      renderMeta(store.meta, false);
    };

    es.addEventListener("init", (e) => {
      store.sseConnected = true;
      applyFullSnapshot(JSON.parse(e.data));
    });

    es.addEventListener("telemetry", (e) => {
      store.sseConnected = true;
      appendPacket(JSON.parse(e.data));
    });

    es.addEventListener("status", (e) => {
      const payload = JSON.parse(e.data);
      if (payload.event) {
        store.events.push(payload.event);
        if (store.events.length > 160) {
          store.events.splice(0, store.events.length - 160);
        }
        renderEvents();
      }
      renderMeta(payload.meta, true);
    });

    es.addEventListener("command", (e) => {
      const payload = JSON.parse(e.data);
      if (Array.isArray(payload.commands)) {
        store.commands = payload.commands;
        renderCommands();
      }
      if (payload.event) {
        store.events.push(payload.event);
        if (store.events.length > 160) {
          store.events.splice(0, store.events.length - 160);
        }
        renderEvents();
      }
      if (payload.meta) renderMeta(payload.meta, false);
    });

    es.addEventListener("reset", (e) => {
      const payload = JSON.parse(e.data);
      store.packets = [];
      store.barcodes = [];
      store.stateTransitions = [];
      store.lastRxMs = null;
      store.events = payload.event ? [payload.event] : [];
      renderMeta(payload.meta, false);
      renderLatestPacket(null);
      renderStateTransitions();
      renderBarcodes();
      renderCharts();
      renderLogs();
      renderEvents();
    });
  }

  function startAgeTicker() {
    setInterval(() => {
      if (store.lastRxMs !== null) {
        const ageSec = Math.max(0, (Date.now() - store.lastRxMs) / 1000);
        ui.kpiAge.textContent = `${ageSec.toFixed(1)} s`;
      } else {
        ui.kpiAge.textContent = "--";
      }
      updateConnectionPill();
    }, 250);
  }

  wireControls();
  startStream();
  startAgeTicker();
})();
