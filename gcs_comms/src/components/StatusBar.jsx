export default function StatusBar({ connected, state }) {
  const gpsFix = state.gps?.fix || 'NO DATA'
  const navStatus = state.nav?.status || 'NO DATA'
  const speed = state.odom?.vx ?? 0
  const heading = state.odom?.yaw ?? 0

  return (
    <div className="flex items-center justify-between px-4 py-2 bg-black border-b border-zinc-800 shrink-0">
      <div className="flex items-center gap-3">
        <span className="text-sm font-black tracking-widest text-white">MERCURY BASESTATION</span>
        <span className="text-xs text-zinc-500">UGVC-10</span>
      </div>
      <div className="flex items-center gap-4 text-xs">
        <span>NAV <span className="text-sky-300 font-bold">{navStatus}</span></span>
        <span>GPS <span className="text-emerald-300 font-bold">{gpsFix}</span></span>
        <span>VX <span className="text-yellow-300 font-bold">{Number(speed).toFixed(2)} m/s</span></span>
        <span>HDG <span className="text-white font-bold">{Number(heading).toFixed(1)} deg</span></span>
        <div className="flex items-center gap-1">
          <div className={`w-2 h-2 rounded-full ${connected ? 'bg-emerald-400 animate-pulse' : 'bg-red-500'}`} />
          <span className={connected ? 'text-emerald-300' : 'text-red-300'}>{connected ? 'LINKED' : 'NO LINK'}</span>
        </div>
      </div>
    </div>
  )
}
