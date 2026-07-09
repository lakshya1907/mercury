import { useEffect, useRef } from "react"

export default function AlertLog({ alerts }) {
  const colors = {
    success: "text-green-400",
    warn: "text-yellow-400",
    danger: "text-red-400",
    info: "text-zinc-400",
  }

  const dots = {
    success: "bg-green-400",
    warn: "bg-yellow-400",
    danger: "bg-red-400",
    info: "bg-zinc-600",
  }

  const endRef = useRef(null)

  // Auto-scroll to latest
  useEffect(() => {
    endRef.current?.scrollIntoView({ behavior: "smooth" })
  }, [alerts])

  return (
    <div className="h-full bg-zinc-900 border border-zinc-800 rounded-lg p-3 flex flex-col overflow-hidden">
      <div className="text-xs text-zinc-500 font-bold tracking-widest mb-2 shrink-0">
        EVENT LOG
      </div>

      <div className="flex-1 overflow-y-auto flex flex-col gap-1 min-h-0">
        {alerts.length === 0 && (
          <span className="text-xs text-zinc-700">
            Waiting for events...
          </span>
        )}

        {alerts.map((a) => (
          <div
            key={a.id}
            className="flex items-start gap-2 text-xs py-0.5 border-b border-zinc-800/50"
          >
            {/* dot */}
            <div
              className={`w-1.5 h-1.5 rounded-full mt-1 shrink-0 ${
                dots[a.type] || "bg-zinc-600"
              }`}
            />

            {/* timestamp */}
            <span className="text-zinc-400 shrink-0 font-mono w-[90px]">
  {a.time}
</span>

            {/* message */}
            <span className={`${colors[a.type] || 'text-zinc-400'} leading-tight`}>
  [{a.type.toUpperCase()}] {a.msg}
</span>
          </div>
        ))}

        <div ref={endRef} />
      </div>
    </div>
  )
}
