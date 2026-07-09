import { useRef, useState, useEffect, useCallback } from 'react'

const VX_MAX = 0.5
const WZ_MAX = 1.5

export default function Joystick({ sendCommand, drive }) {
  const canvasRef = useRef(null)
  const dragging = useRef(false)
  const stick = useRef({ x: 0, y: 0 })
  const [active, setActive] = useState(false)
  const [output, setOutput] = useState({ linear: 0, angular: 0 })
  const center = 50
  const max = 35

  const getPos = (event, canvas) => {
    const rect = canvas.getBoundingClientRect()
    const point = event.touches ? event.touches[0] : event
    return { x: point.clientX - rect.left, y: point.clientY - rect.top }
  }

  const draw = useCallback(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const ctx = canvas.getContext('2d')
    ctx.clearRect(0, 0, 100, 100)

    ctx.beginPath()
    ctx.arc(center, center, max + 8, 0, Math.PI * 2)
    ctx.strokeStyle = '#3f3f46'
    ctx.lineWidth = 1.5
    ctx.stroke()

    ctx.strokeStyle = '#27272a'
    ctx.lineWidth = 0.75
    ctx.beginPath()
    ctx.moveTo(center, center - max)
    ctx.lineTo(center, center + max)
    ctx.stroke()
    ctx.beginPath()
    ctx.moveTo(center - max, center)
    ctx.lineTo(center + max, center)
    ctx.stroke()

    ctx.beginPath()
    ctx.arc(center + stick.current.x, center + stick.current.y, 14, 0, Math.PI * 2)
    ctx.fillStyle = active ? '#38bdf8' : '#52525b'
    ctx.fill()
    ctx.strokeStyle = active ? '#0ea5e9' : '#3f3f46'
    ctx.lineWidth = 2
    ctx.stroke()
  }, [active])

  useEffect(() => {
    draw()
  }, [draw])

  const sendVelocity = useCallback((linear, angular) => {
    const rounded = {
      linear: Number(linear.toFixed(3)),
      angular: Number(angular.toFixed(3)),
    }
    setOutput(rounded)
    sendCommand({
      type: 'cmd_vel',
      linear_ms: rounded.linear,
      angular_rads: rounded.angular,
    })
  }, [sendCommand])

  const onStart = (event) => {
    event.preventDefault()
    dragging.current = true
    setActive(true)
  }

  const onMove = useCallback((event) => {
    if (!dragging.current || !canvasRef.current) return
    event.preventDefault()
    const pos = getPos(event, canvasRef.current)
    let dx = pos.x - center
    let dy = pos.y - center
    const dist = Math.sqrt(dx * dx + dy * dy)
    if (dist > max) {
      dx = (dx / dist) * max
      dy = (dy / dist) * max
    }

    stick.current = { x: dx, y: dy }
    sendVelocity(-(dy / max) * VX_MAX, -(dx / max) * WZ_MAX)
    draw()
  }, [draw, sendVelocity])

  const onEnd = useCallback(() => {
    if (!dragging.current) return
    dragging.current = false
    setActive(false)
    stick.current = { x: 0, y: 0 }
    sendVelocity(0, 0)
    draw()
  }, [draw, sendVelocity])

  useEffect(() => {
    window.addEventListener('mousemove', onMove)
    window.addEventListener('mouseup', onEnd)
    window.addEventListener('touchmove', onMove, { passive: false })
    window.addEventListener('touchend', onEnd)
    return () => {
      window.removeEventListener('mousemove', onMove)
      window.removeEventListener('mouseup', onEnd)
      window.removeEventListener('touchmove', onMove)
      window.removeEventListener('touchend', onEnd)
    }
  }, [onMove, onEnd])

  return (
    <div className="bg-zinc-950 border border-zinc-800 rounded-lg p-3 flex items-center gap-4 min-w-0">
      <div className="flex flex-col items-center gap-1 shrink-0">
        <span className="text-[11px] text-zinc-500 uppercase tracking-widest font-black">Drive</span>
        <canvas
          ref={canvasRef}
          width={100}
          height={100}
          onMouseDown={onStart}
          onTouchStart={onStart}
          className="cursor-pointer"
          style={{ touchAction: 'none' }}
        />
      </div>

      <div className="flex-1 min-w-0 grid grid-cols-2 gap-2 text-xs">
        <div className="bg-zinc-900 border border-zinc-800 rounded-md p-2">
          <div className="text-zinc-500">Linear</div>
          <div className="text-lg font-mono font-black text-white">{output.linear.toFixed(2)}</div>
          <div className="text-zinc-600">m/s</div>
        </div>
        <div className="bg-zinc-900 border border-zinc-800 rounded-md p-2">
          <div className="text-zinc-500">Angular</div>
          <div className="text-lg font-mono font-black text-white">{output.angular.toFixed(2)}</div>
          <div className="text-zinc-600">rad/s</div>
        </div>
        <div className="col-span-2 bg-zinc-900 border border-zinc-800 rounded-md p-2 flex items-center justify-between">
          <span className="text-zinc-500">Rover echo</span>
          <span className="font-mono text-white">
            {(drive?.rover_vx ?? 0).toFixed(2)} / {(drive?.rover_wz ?? 0).toFixed(2)}
          </span>
        </div>
      </div>
    </div>
  )
}
