const fmt = (value, digits = 1, unit = '') => (
  value === null || value === undefined ? 'NO DATA' : `${Number(value).toFixed(digits)}${unit}`
)

function Pill({ active, children, good = true }) {
  const on = good ? 'bg-emerald-500/10 text-emerald-300 border-emerald-500/30' : 'bg-red-500/10 text-red-300 border-red-500/30'
  return (
    <span className={`px-2 py-1 rounded border text-[11px] font-black ${active ? on : 'bg-zinc-900 text-zinc-500 border-zinc-800'}`}>
      {children}
    </span>
  )
}

export default function DetectionPanel({ lane, face, ages = {} }) {
  const driftTone = lane.drift === 'CENTRE' ? 'text-emerald-300' : lane.drift === 'NO DATA' ? 'text-zinc-500' : 'text-amber-300'

  return (
    <div className="bg-zinc-950 border border-zinc-800 rounded-lg p-3 flex flex-col gap-3 shrink-0">
      <div className="flex items-center justify-between">
        <span className="text-[11px] text-zinc-500 font-black tracking-widest uppercase">Perception</span>
        <span className="text-[10px] text-zinc-600 font-mono">lane {ages.lane ?? 'wait'}s / face {ages.face ?? 'wait'}s</span>
      </div>

      <div className="bg-zinc-900 border border-zinc-800 rounded-md p-2">
        <div className="flex items-center justify-between mb-2">
          <span className="text-xs font-bold text-zinc-300">Lane Detection</span>
          <span className={`text-xs font-black ${driftTone}`}>{lane.drift}</span>
        </div>
        <div className="flex flex-wrap gap-2 mb-2">
          <Pill active={lane.visible}>VISIBLE</Pill>
          <Pill active={lane.both_visible}>BOTH LANES</Pill>
        </div>
        <div className="flex justify-between text-xs">
          <span className="text-zinc-500">Center error</span>
          <span className="font-mono text-white">{fmt(lane.error_px, 1, ' px')}</span>
        </div>
      </div>

      <div className="bg-zinc-900 border border-zinc-800 rounded-md p-2">
        <div className="flex items-center justify-between mb-2">
          <span className="text-xs font-bold text-zinc-300">Face / Turret</span>
          <span className={face.complete ? 'text-emerald-300 text-xs font-black' : 'text-zinc-500 text-xs font-black'}>
            {face.complete ? 'DONE' : 'TRACKING'}
          </span>
        </div>
        <div className="flex flex-wrap gap-2 mb-2">
          <Pill active={face.active}>ACTIVE</Pill>
          <Pill active={face.match}>MATCH</Pill>
          <Pill active={face.complete}>COMPLETE</Pill>
        </div>
        <div className="grid grid-cols-2 gap-2 text-xs">
          <div className="flex justify-between gap-2">
            <span className="text-zinc-500">H error</span>
            <span className="font-mono text-white">{fmt(face.h_error_px, 1, ' px')}</span>
          </div>
          <div className="flex justify-between gap-2">
            <span className="text-zinc-500">V error</span>
            <span className="font-mono text-white">{fmt(face.v_error_px, 1, ' px')}</span>
          </div>
        </div>
      </div>
    </div>
  )
}
