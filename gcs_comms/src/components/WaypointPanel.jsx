const fmt = (value, digits = 2, unit = '') => (
  value === null || value === undefined ? 'NO DATA' : `${Number(value).toFixed(digits)}${unit}`
)

function Row({ label, value, tone = 'text-white' }) {
  return (
    <div className="flex items-center justify-between gap-3 text-xs border-b border-zinc-800/60 pb-1 last:border-0 last:pb-0">
      <span className="text-zinc-500">{label}</span>
      <span className={`font-mono text-right truncate ${tone}`}>{value}</span>
    </div>
  )
}

export default function WaypointPanel({ mission, nav, encoders }) {
  const statusTone = {
    EXECUTING: 'text-emerald-300',
    SUCCEEDED: 'text-sky-300',
    ABORTED: 'text-red-300',
    CANCELED: 'text-amber-300',
  }[nav.status] || 'text-zinc-300'

  return (
    <div className="bg-zinc-950 border border-zinc-800 rounded-lg p-3 flex flex-col gap-3 shrink-0">
      <div className="flex items-center justify-between">
        <span className="text-[11px] text-zinc-500 font-black tracking-widest uppercase">Navigation and Mission</span>
        <span className={`text-[11px] font-black ${mission.all_done ? 'text-emerald-300' : 'text-zinc-500'}`}>
          {mission.all_done ? 'ALL COMPLETE' : 'IN PROGRESS'}
        </span>
      </div>

      <div className="grid grid-cols-2 gap-3">
        <div className="bg-zinc-900 border border-zinc-800 rounded-md p-2 flex flex-col gap-1">
          <Row label="Nav status" value={nav.status} tone={statusTone} />
          <Row label="Goal X" value={fmt(nav.goal_x, 3, ' m')} />
          <Row label="Goal Y" value={fmt(nav.goal_y, 3, ' m')} />
        </div>

        <div className="bg-zinc-900 border border-zinc-800 rounded-md p-2 flex flex-col gap-1">
          <Row label="Last WP" value={mission.wp_name} />
          <Row label="Index" value={mission.wp_idx} />
          <Row label="Distance" value={fmt(mission.wp_dist, 2, ' m')} />
        </div>
      </div>

      <div className="bg-zinc-900 border border-zinc-800 rounded-md p-2">
        <div className="flex items-center justify-between mb-1">
          <span className="text-xs font-bold text-zinc-300">Encoders</span>
          <span className="text-[11px] text-zinc-500">{encoders.names?.length || 0} joints</span>
        </div>
        <div className="grid grid-cols-2 gap-x-3 gap-y-1 max-h-16 overflow-y-auto">
          {(encoders.names || []).slice(0, 8).map((name, index) => (
            <div key={`${name}-${index}`} className="flex justify-between gap-2 text-[11px]">
              <span className="text-zinc-500 truncate">{name}</span>
              <span className="text-white font-mono">{fmt(encoders.velocity?.[index], 2)}</span>
            </div>
          ))}
          {(!encoders.names || encoders.names.length === 0) && (
            <span className="text-xs text-zinc-600">Waiting for encoder packets</span>
          )}
        </div>
      </div>
    </div>
  )
}
