export default function AlertsConnection({ alerts, connected, rx, system }) {
  const colors = {
    success: 'bg-emerald-500/10 text-emerald-300 border-emerald-500/20',
    warn: 'bg-amber-500/10 text-amber-300 border-amber-500/20',
    danger: 'bg-red-500/10 text-red-300 border-red-500/20',
    info: 'bg-zinc-900 text-zinc-400 border-zinc-800',
  }

  const counters = [
    ['IMU', rx?.imu],
    ['GPS', rx?.gps],
    ['ODOM', rx?.odom],
    ['LANE', rx?.lane],
    ['FACE', rx?.face],
    ['VID', rx?.video],
  ]

  return (
    <div className="h-full bg-zinc-950 border border-zinc-800 rounded-lg p-3 flex flex-col gap-3 min-h-0 overflow-hidden">
      <div className="flex items-center justify-between shrink-0">
        <span className="text-[11px] text-zinc-500 font-black tracking-widest uppercase">Alerts and Link</span>
        <span className={`text-[11px] font-black ${connected ? 'text-emerald-300' : 'text-red-300'}`}>
          {connected ? 'CONNECTED' : 'DISCONNECTED'}
        </span>
      </div>

      <div className="grid grid-cols-3 gap-2 shrink-0">
        {counters.map(([label, value]) => (
          <div key={label} className="bg-zinc-900 border border-zinc-800 rounded-md px-2 py-1">
            <div className="text-[10px] text-zinc-500 font-bold">{label}</div>
            <div className="text-sm text-white font-mono font-black">{value ?? 0}</div>
          </div>
        ))}
      </div>

      <div className="grid grid-cols-2 gap-2 shrink-0 text-xs">
        <div className="bg-zinc-900 border border-zinc-800 rounded-md p-2 flex justify-between">
          <span className="text-zinc-500">CPU</span>
          <span className="text-white font-mono">{system?.cpu_pct ?? 'NO DATA'}%</span>
        </div>
        <div className="bg-zinc-900 border border-zinc-800 rounded-md p-2 flex justify-between">
          <span className="text-zinc-500">RAM</span>
          <span className="text-white font-mono">{system?.mem_pct ?? 'NO DATA'}%</span>
        </div>
      </div>

      <div className="flex flex-col gap-1.5 overflow-y-auto flex-1 min-h-0">
        {alerts.length === 0 && (
          <div className="text-xs text-zinc-700 border border-zinc-800 rounded-md px-2 py-2">Waiting for alerts</div>
        )}
        {alerts.slice(0, 14).map((alert) => (
          <div key={alert.id} className={`text-xs px-2 py-1.5 rounded-md border ${colors[alert.type] || colors.info}`}>
            <span className="font-mono text-zinc-500 mr-2">{alert.time}</span>
            {alert.msg}
          </div>
        ))}
      </div>
    </div>
  )
}
