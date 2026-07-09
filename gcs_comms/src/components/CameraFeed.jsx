const VIDEO_BASE = import.meta.env.VITE_VIDEO_BASE_URL || `http://${window.location.hostname || 'localhost'}:8080`

export default function CameraFeed({ title, stream, age }) {
  const src = `${VIDEO_BASE}/${stream}.mjpg`
  const live = Number.isFinite(age) && age < 3

  return (
    <div className="bg-zinc-950 border border-zinc-800 rounded-lg p-2">
      <div className="flex items-center justify-between mb-2">
        <span className="text-[11px] text-zinc-500 font-black tracking-widest uppercase">{title}</span>
        <span className={`text-[10px] font-bold ${live ? 'text-emerald-300' : 'text-zinc-600'}`}>
          {live ? `${age.toFixed(1)}s` : 'WAITING'}
        </span>
      </div>

      <div className="aspect-video bg-black rounded-md overflow-hidden relative flex items-center justify-center border border-zinc-900">
        <span className="text-xs text-zinc-700 absolute">NO FEED</span>
        <img
          src={src}
          alt={`${title} stream`}
          className="relative z-10 w-full h-full object-cover"
        />
        <div className="absolute left-2 top-2 z-20 flex items-center gap-1.5 bg-black/70 rounded px-2 py-1">
          <span className={`w-1.5 h-1.5 rounded-full ${live ? 'bg-red-500 animate-pulse' : 'bg-zinc-600'}`} />
          <span className="text-[10px] text-zinc-300 font-bold">{stream.toUpperCase()}</span>
        </div>
      </div>
    </div>
  )
}
