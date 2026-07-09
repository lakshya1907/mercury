import { MapContainer, TileLayer, Marker, Circle, Polyline, useMap } from 'react-leaflet'
import { useEffect } from 'react'
import L from 'leaflet'

const ORIGIN = [30.0808, 31.2969]

const toLatLng = (x = 0, y = 0) => [
  ORIGIN[0] + (y / 111320),
  ORIGIN[1] + (x / 111320),
]

const robotIcon = new L.DivIcon({
  className: '',
  html: '<div style="width:18px;height:18px;background:#38bdf8;border:3px solid #082f49;border-radius:50%;box-shadow:0 0 0 2px #38bdf8;"></div>',
  iconSize: [18, 18],
  iconAnchor: [9, 9],
})

const goalIcon = new L.DivIcon({
  className: '',
  html: '<div style="width:14px;height:14px;background:#fbbf24;border:2px solid #451a03;transform:rotate(45deg);"></div>',
  iconSize: [14, 14],
  iconAnchor: [7, 7],
})

function FollowRobot({ position }) {
  const map = useMap()
  useEffect(() => {
    map.setView(position, map.getZoom(), { animate: true })
  }, [map, position])
  return null
}

export default function MapPanel({ state }) {
  const gps = state.gps || {}
  const odom = state.odom || {}
  const nav = state.nav || {}
  const hasGps = Number.isFinite(gps.lat) && Number.isFinite(gps.lon)
  const robotPos = hasGps ? [gps.lat, gps.lon] : toLatLng(odom.x || 0, odom.y || 0)
  const hasGoal = Number.isFinite(nav.goal_x) && Number.isFinite(nav.goal_y)
  const goalPos = hasGoal ? toLatLng(nav.goal_x, nav.goal_y) : null

  return (
    <div className="h-full w-full relative">
      <MapContainer
        center={robotPos}
        zoom={18}
        minZoom={17}
        maxZoom={19}
        style={{ height: '100%', width: '100%', background: '#09090b' }}
        scrollWheelZoom
      >
        <TileLayer
          url="http://localhost:8000/{z}/{x}/{y}.png"
          minZoom={17}
          maxZoom={19}
        />

        <FollowRobot position={robotPos} />
        <Marker position={robotPos} icon={robotIcon} />
        <Circle center={robotPos} radius={2.5} pathOptions={{ color: '#67e8f9', weight: 2 }} />

        {goalPos && (
          <>
            <Marker position={goalPos} icon={goalIcon} />
            <Polyline positions={[robotPos, goalPos]} pathOptions={{ color: '#fbbf24', weight: 2, dashArray: '5 6' }} />
          </>
        )}
      </MapContainer>

      <div className="absolute left-3 top-3 z-[500] bg-black/80 border border-zinc-700 rounded-md px-3 py-2">
        <div className="text-[10px] text-zinc-500 font-black tracking-widest uppercase">Map Source</div>
        <div className="text-xs text-white font-bold">{hasGps ? 'GPS fix' : 'Odometry local frame'}</div>
      </div>
    </div>
  )
}
