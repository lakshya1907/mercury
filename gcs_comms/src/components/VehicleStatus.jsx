const fmt = (value, digits = 2, unit = '') => {
  if (value === null || value === undefined || Number.isNaN(value)) return 'NO DATA'
  return `${Number(value).toFixed(digits)}${unit}`
}

const staleClass = (age) => {
  if (age === null || age === undefined) return 'text-zinc-600'
  if (age > 3) return 'text-amber-300'
  return 'text-emerald-300'
}

function Tile({ label, value, sub, tone = 'text-white' }) {
  return (
    <div className="bg-zinc-900 border border-zinc-800 rounded-md p-2 min-w-0">
      <div className="text-[10px] text-zinc-500 font-bold tracking-widest uppercase">{label}</div>
      <div className={`text-lg font-black leading-tight truncate ${tone}`}>{value}</div>
      {sub && <div className="text-[11px] text-zinc-500 truncate">{sub}</div>}
    </div>
  )
}

function Section({ title, age, children }) {
  return (
    <section className="flex flex-col gap-2">
      <div className="flex items-center justify-between">
        <span className="text-[11px] text-zinc-500 font-black tracking-widest uppercase">{title}</span>
        <span className={`text-[10px] font-mono ${staleClass(age)}`}>
          {age === null || age === undefined ? 'WAITING' : `${age.toFixed(1)}s`}
        </span>
      </div>
      {children}
    </section>
  )
}

export default function VehicleStatus({ state }) {
  const { drive, odom, imu, gps, lidar, system, health, ages = {} } = state
  const healthText = health.all_ok === null ? 'NO DATA' : health.all_ok ? 'OK' : 'CHECK'
  const healthTone = health.all_ok === false ? 'text-red-300' : health.all_ok ? 'text-emerald-300' : 'text-zinc-500'

  return (
    <div className="min-h-0 flex-1 bg-zinc-950 border border-zinc-800 rounded-lg p-3 flex flex-col gap-4 overflow-y-auto">
      <Section title="Drive Command" age={ages.odom}>
        <div className="grid grid-cols-2 gap-2">
          <Tile label="Base VX" value={fmt(drive.vx, 2, ' m/s')} tone={drive.vx > 0 ? 'text-emerald-300' : drive.vx < 0 ? 'text-red-300' : 'text-white'} />
          <Tile label="Base WZ" value={fmt(drive.wz, 2, ' rad/s')} tone={drive.wz ? 'text-sky-300' : 'text-white'} />
          <Tile label="Rover VX" value={fmt(drive.rover_vx, 3, ' m/s')} />
          <Tile label="Rover WZ" value={fmt(drive.rover_wz, 3, ' rad/s')} />
        </div>
      </Section>

      <Section title="Odometry" age={ages.odom}>
        <div className="grid grid-cols-3 gap-2">
          <Tile label="X" value={fmt(odom.x, 3, ' m')} />
          <Tile label="Y" value={fmt(odom.y, 3, ' m')} />
          <Tile label="Yaw" value={fmt(odom.yaw, 1, ' deg')} />
          <Tile label="Linear" value={fmt(odom.vx, 3, ' m/s')} />
          <Tile label="Angular" value={fmt(odom.wz, 3, ' rad/s')} />
          <Tile label="Front" value={fmt(lidar.front, 2, ' m')} tone={(lidar.front ?? 99) < 1 ? 'text-red-300' : 'text-white'} />
        </div>
      </Section>

      <Section title="IMU" age={ages.imu}>
        <div className="grid grid-cols-3 gap-2">
          <Tile label="Roll" value={fmt(imu.roll, 1, ' deg')} />
          <Tile label="Pitch" value={fmt(imu.pitch, 1, ' deg')} />
          <Tile label="Yaw" value={fmt(imu.yaw, 1, ' deg')} />
          <Tile label="Accel X" value={fmt(imu.ax, 2)} />
          <Tile label="Accel Y" value={fmt(imu.ay, 2)} />
          <Tile label="Gyro Z" value={fmt(imu.wz, 3)} />
        </div>
      </Section>

      <Section title="GPS" age={ages.gps}>
        <div className="grid grid-cols-2 gap-2">
          <Tile label="Fix" value={gps.fix} tone={String(gps.fix).includes('FIX') || gps.fix === 'RTK' ? 'text-emerald-300' : 'text-red-300'} />
          <Tile label="Altitude" value={fmt(gps.alt, 1, ' m')} />
          <Tile label="Latitude" value={fmt(gps.lat, 7)} />
          <Tile label="Longitude" value={fmt(gps.lon, 7)} />
        </div>
      </Section>

      <Section title="LiDAR and Rover System" age={ages.system}>
        <div className="grid grid-cols-2 gap-2">
          <Tile label="Min Range" value={fmt(lidar.min, 2, ' m')} />
          <Tile label="Mean Range" value={fmt(lidar.mean, 2, ' m')} />
          <Tile label="CPU" value={fmt(system.cpu_pct, 1, '%')} />
          <Tile label="Memory" value={fmt(system.mem_pct, 1, '%')} sub={`${fmt(system.mem_used_mb, 0, ' MB')} used`} />
          <Tile label="Health" value={healthText} tone={healthTone} />
          <Tile label="Missing Nodes" value={health.missing?.length || 0} sub={(health.missing || []).slice(0, 2).join(', ')} />
        </div>
      </Section>
    </div>
  )
}
