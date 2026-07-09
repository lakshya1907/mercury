const fmt = (value, digits = 2, unit = '') => (
  value === null || value === undefined ? 'NO DATA' : `${Number(value).toFixed(digits)}${unit}`
)

export default function EStopButton({ onEStop, health, lidar }) {
  const front = lidar?.front
  const close = Number.isFinite(front) && front < 1
  const healthBad = health?.all_ok === false

  return (
    <div className="bg-zinc-950 border border-zinc-800 rounded-lg p-3 flex flex-col gap-2">
      <div className={`text-xs text-center py-1.5 rounded border font-black tracking-wider ${
        close ? 'bg-red-500/10 text-red-300 border-red-500/30' : 'bg-zinc-900 text-zinc-500 border-zinc-800'
      }`}>
        FRONT {fmt(front, 2, ' m')}
      </div>

      <div className={`text-xs text-center py-1.5 rounded border font-black tracking-wider ${
        healthBad ? 'bg-amber-500/10 text-amber-300 border-amber-500/30' : 'bg-zinc-900 text-zinc-500 border-zinc-800'
      }`}>
        {healthBad ? 'SYSTEM CHECK' : 'SYSTEM OK'}
      </div>

      <button
        onClick={onEStop}
        className="w-full flex-1 min-h-16 bg-red-600 hover:bg-red-500 active:scale-95 active:bg-red-700 transition-all rounded-md font-black text-white tracking-widest text-base border-2 border-red-400/50 shadow-lg"
      >
        E-STOP
      </button>
    </div>
  )
}
