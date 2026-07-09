import { useState, useEffect, useRef, useCallback } from 'react'

const WS_URL = import.meta.env.VITE_WS_URL || `ws://${window.location.hostname || 'localhost'}:9090`

const EMPTY_STATE = {
  mode: 'MANUAL',
  pose: { x: 0, y: 0, yaw: 0 },
  speed: 0,
  drive: { vx: 0, wz: 0, rover_vx: null, rover_wz: null, ts: 0, rover_ts: 0 },
  imu: { roll: null, pitch: null, yaw: null, ax: null, ay: null, az: null, wx: null, wy: null, wz: null, ts: 0 },
  gps: { lat: null, lon: null, alt: null, fix: 'NO DATA', ts: 0 },
  odom: { x: null, y: null, yaw: null, vx: null, wz: null, ts: 0 },
  encoders: { names: [], position: [], velocity: [], ts: 0 },
  lidar: { front: null, min: null, mean: null, max: null, ts: 0 },
  nav: { goal_x: null, goal_y: null, status: 'NO DATA', ts: 0 },
  system: { cpu_pct: null, mem_pct: null, mem_used_mb: null, ts: 0 },
  lane: { error_px: null, visible: false, both_visible: false, drift: 'NO DATA', ts: 0 },
  face: { active: false, match: false, h_error_px: null, v_error_px: null, complete: false, ts: 0 },
  mission: { wp_name: 'NO DATA', wp_idx: 'NO DATA', wp_dist: null, all_done: false, ts: 0 },
  health: { all_ok: null, missing: [], ts: 0 },
  alerts: [],
  rx: { imu: 0, gps: 0, odom: 0, enc: 0, lane: 0, face: 0, mission: 0, alert: 0, video: 0 },
  video: { lane_ts: 0, turret_ts: 0 },
  ages: {},
  costmap: null,
  waypoints: [],
}

const section = (defaults, incoming) => ({ ...defaults, ...(incoming || {}) })

function normalizeState(data, prev = EMPTY_STATE) {
  if (data?.imu || data?.gps || data?.odom || data?.lane || data?.face) {
    const odom = section(EMPTY_STATE.odom, data.odom)
    const yawDeg = Number.isFinite(odom.yaw) ? odom.yaw : 0
    return {
      ...prev,
      ...data,
      mode: data.mode || prev.mode || 'MANUAL',
      drive: section(EMPTY_STATE.drive, data.drive),
      imu: section(EMPTY_STATE.imu, data.imu),
      gps: section(EMPTY_STATE.gps, data.gps),
      odom,
      encoders: section(EMPTY_STATE.encoders, data.encoders),
      lidar: section(EMPTY_STATE.lidar, data.lidar),
      nav: section(EMPTY_STATE.nav, data.nav),
      system: section(EMPTY_STATE.system, data.system),
      lane: section(EMPTY_STATE.lane, data.lane),
      face: section(EMPTY_STATE.face, data.face),
      mission: section(EMPTY_STATE.mission, data.mission),
      health: section(EMPTY_STATE.health, data.health),
      alerts: Array.isArray(data.alerts) ? data.alerts : [],
      rx: section(EMPTY_STATE.rx, data.rx),
      video: section(EMPTY_STATE.video, data.video),
      ages: data.ages || {},
      pose: {
        x: odom.x ?? 0,
        y: odom.y ?? 0,
        yaw: yawDeg * Math.PI / 180,
      },
      speed: Math.abs(odom.vx ?? 0),
      waypoints: data.waypoints || prev.waypoints || [],
      costmap: data.costmap || prev.costmap || null,
    }
  }

  const pose = data?.pose || prev.pose || EMPTY_STATE.pose
  return {
    ...prev,
    pose,
    speed: data?.speed ?? prev.speed ?? 0,
    mode: data?.mode || prev.mode || 'MANUAL',
    waypoints: data?.waypoints || prev.waypoints || [],
    costmap: data?.costmap || prev.costmap || null,
    odom: {
      ...prev.odom,
      x: pose.x,
      y: pose.y,
      yaw: Number.isFinite(pose.yaw) ? pose.yaw * 180 / Math.PI : prev.odom.yaw,
      vx: data?.speed ?? prev.odom.vx,
    },
  }
}

export function useRobot() {
  const [connected, setConnected] = useState(false)
  const [state, setState] = useState(EMPTY_STATE)
  const [localAlerts, setLocalAlerts] = useState([])
  const ws = useRef(null)
  const reconnectTimer = useRef(null)

  const addAlert = useCallback((msg, type = 'info') => {
    const entry = { id: `${Date.now()}-${Math.random()}`, msg, type, time: new Date().toLocaleTimeString() }
    setLocalAlerts(a => [entry, ...a].slice(0, 20))
  }, [])

  useEffect(() => {
    let stopped = false

    function connect() {
      ws.current = new WebSocket(WS_URL)

      ws.current.onopen = () => {
        setConnected(true)
        addAlert('Connected to Mercury bridge', 'success')
      }

      ws.current.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data)
          setState(prev => {
            const next = normalizeState(data, prev)
            if (next.lane.visible && !prev.lane.visible) addAlert('Lane detected', 'success')
            if (!next.lane.visible && prev.lane.visible) addAlert('Lane lost', 'warn')
            if (next.face.match && !prev.face.match) addAlert('Face match found', 'success')
            if (next.health.all_ok === false && prev.health.all_ok !== false) addAlert('Rover health warning', 'danger')
            return next
          })
        } catch (err) {
          console.error('WS parse error', err)
        }
      }

      ws.current.onclose = () => {
        setConnected(false)
        if (!stopped) {
          addAlert('Bridge disconnected, retrying', 'danger')
          reconnectTimer.current = setTimeout(connect, 2000)
        }
      }

      ws.current.onerror = () => {
        ws.current?.close()
      }
    }

    connect()
    return () => {
      stopped = true
      clearTimeout(reconnectTimer.current)
      ws.current?.close()
    }
  }, [addAlert])

  const sendCommand = useCallback((cmd) => {
    if (ws.current?.readyState === WebSocket.OPEN) {
      ws.current.send(JSON.stringify(cmd))
    }
  }, [])

  const eStop = useCallback(() => {
    sendCommand({ type: 'ESTOP' })
    addAlert('E-STOP triggered', 'danger')
  }, [sendCommand, addAlert])

  const alerts = [...(state.alerts || []), ...localAlerts].slice(0, 30)

  return { connected, state, alerts, sendCommand, eStop, wsUrl: WS_URL }
}
