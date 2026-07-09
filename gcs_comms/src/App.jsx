import { useRobot } from './hooks/useRobot'
import MapPanel from './components/MapPanel'
import CameraFeed from './components/CameraFeed'
import EStopButton from './components/EStopButton'
import DetectionPanel from './components/DetectionPanel'
import VehicleStatus from './components/VehicleStatus'
import WaypointPanel from './components/WaypointPanel'
import AlertsConnection from './components/AlertsConnection'
import Joystick from './components/Joystick'

export default function App() {
  const { connected, state, alerts, eStop, sendCommand } = useRobot()
  const navStatus = state.nav?.status || 'NO DATA'
  const gpsFix = state.gps?.fix || 'NO DATA'

  return (
    <div className="h-screen w-screen bg-neutral-950 text-zinc-100 flex flex-col overflow-hidden font-sans">
      <div className="flex items-center justify-between px-5 py-3 bg-black border-b border-zinc-800 shrink-0">
        <div className="flex items-center gap-3">
          <span className="font-black text-base tracking-wide text-white">Mercury Base Station</span>
          <span className="text-xs text-zinc-500">192.168.88.2</span>
        </div>

        <div className="flex items-center gap-3 text-xs">
          <span className={`px-2.5 py-1 rounded border font-bold ${
            connected ? 'bg-emerald-500/10 text-emerald-300 border-emerald-500/30' : 'bg-red-500/10 text-red-300 border-red-500/30'
          }`}>
            {connected ? 'BRIDGE LIVE' : 'NO BRIDGE'}
          </span>
          <span className="px-2.5 py-1 rounded border border-zinc-700 bg-zinc-900 text-zinc-300 font-bold">
            GPS {gpsFix}
          </span>
          <span className="px-2.5 py-1 rounded border border-sky-500/30 bg-sky-500/10 text-sky-300 font-bold">
            NAV {navStatus}
          </span>
          <span className="text-zinc-500 font-mono">{new Date().toLocaleTimeString()}</span>
        </div>
      </div>

      <div
        className="flex-1 min-h-0 overflow-hidden p-3 grid gap-3"
        style={{ gridTemplateColumns: '360px minmax(440px, 1fr) 420px' }}
      >
        <div className="min-h-0 flex flex-col gap-3 overflow-hidden">
          <VehicleStatus state={state} />
          <DetectionPanel lane={state.lane} face={state.face} ages={state.ages} />
        </div>

        <div className="min-h-0 flex flex-col gap-3 overflow-hidden">
          <div className="flex-1 min-h-0 rounded-lg overflow-hidden border border-zinc-800 bg-zinc-950">
            <MapPanel state={state} />
          </div>
          <div className="grid grid-cols-[1fr_180px] gap-3 shrink-0">
            <Joystick sendCommand={sendCommand} mode={state.mode} drive={state.drive} />
            <EStopButton onEStop={eStop} health={state.health} lidar={state.lidar} />
          </div>
        </div>

        <div className="min-h-0 flex flex-col gap-3 overflow-hidden">
          <div className="grid grid-cols-2 gap-3 shrink-0">
            <CameraFeed title="Lane Camera" stream="lane" age={state.ages?.lane_video} />
            <CameraFeed title="Turret Camera" stream="turret" age={state.ages?.turret_video} />
          </div>
          <WaypointPanel mission={state.mission} nav={state.nav} encoders={state.encoders} />
          <div className="flex-1 min-h-0 overflow-hidden">
            <AlertsConnection alerts={alerts} connected={connected} rx={state.rx} system={state.system} />
          </div>
        </div>
      </div>
    </div>
  )
}
