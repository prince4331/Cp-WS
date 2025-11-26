import { useRobotStore } from '../store/robotStore';
import { Settings as SettingsIcon, Wifi, Zap, Gauge } from 'lucide-react';

export default function Settings() {
  const maxLinearSpeed = useRobotStore((state) => state.maxLinearSpeed);
  const maxAngularSpeed = useRobotStore((state) => state.maxAngularSpeed);
  const setMaxLinearSpeed = useRobotStore((state) => state.setMaxLinearSpeed);
  const setMaxAngularSpeed = useRobotStore((state) => state.setMaxAngularSpeed);
  const connected = useRobotStore((state) => state.connected);
  const robotStatus = useRobotStore((state) => state.robotStatus);

  return (
    <div className="space-y-3">
      {/* Header */}
      <div>
        <h2 className="text-2xl font-bold text-dark-50">Settings</h2>
        <p className="text-dark-400 text-sm">Configure robot parameters</p>
      </div>

      {/* Speed Limits */}
      <div className="card">
        <h3 className="card-header">
          <Gauge className="w-4 h-4 inline mr-2" />
          Speed Limits
        </h3>
        <div className="space-y-3">
          <div>
            <label className="text-sm text-dark-300 mb-1 block font-medium">
              Maximum Linear Speed: {maxLinearSpeed.toFixed(2)} m/s
            </label>
            <input
              type="range"
              min="0.1"
              max="0.5"
              step="0.05"
              value={maxLinearSpeed}
              onChange={(e) => setMaxLinearSpeed(parseFloat(e.target.value))}
              className="w-full h-2 bg-dark-800 rounded-lg appearance-none cursor-pointer"
            />
            <p className="text-xs text-dark-500 mt-1">
              Controls maximum forward/backward speed. Recommended: 0.3 m/s
            </p>
          </div>

          <div>
            <label className="text-sm text-dark-300 mb-1 block font-medium">
              Maximum Angular Speed: {maxAngularSpeed.toFixed(2)} rad/s
            </label>
            <input
              type="range"
              min="0.5"
              max="2.0"
              step="0.1"
              value={maxAngularSpeed}
              onChange={(e) => setMaxAngularSpeed(parseFloat(e.target.value))}
              className="w-full h-2 bg-dark-800 rounded-lg appearance-none cursor-pointer"
            />
            <p className="text-xs text-dark-500 mt-1">
              Controls maximum rotation speed. Recommended: 1.0 rad/s
            </p>
          </div>
        </div>
      </div>

      {/* Connection Info */}
      <div className="card">
        <h3 className="card-header">
          <Wifi className="w-4 h-4 inline mr-2" />
          Network Information
        </h3>
        <div className="space-y-2">
          <div className="flex justify-between items-center p-2 bg-dark-800 rounded-lg">
            <span className="text-sm text-dark-400">ROS Bridge Status</span>
            <span className={`font-bold ${connected ? 'text-green-500' : 'text-red-500'}`}>
              {connected ? 'Connected' : 'Disconnected'}
            </span>
          </div>
          <div className="flex justify-between items-center p-2 bg-dark-800 rounded-lg">
            <span className="text-sm text-dark-400">WebSocket URL</span>
            <span className="text-xs text-primary-400 font-mono">ws://localhost:9090</span>
          </div>
          <div className="flex justify-between items-center p-2 bg-dark-800 rounded-lg">
            <span className="text-sm text-dark-400">Robot IP Address</span>
            <span className="text-xs text-primary-400 font-mono">192.168.0.166</span>
          </div>
        </div>
      </div>

      {/* System Info */}
      <div className="card">
        <h3 className="card-header">
          <SettingsIcon className="w-4 h-4 inline mr-2" />
          System Information
        </h3>
        <div className="space-y-2">
          <div className="flex justify-between items-center p-2 bg-dark-800 rounded-lg">
            <span className="text-sm text-dark-400">Robot Mode</span>
            <span className="text-sm text-primary-400 font-bold">{robotStatus.mode}</span>
          </div>
          <div className="flex justify-between items-center p-2 bg-dark-800 rounded-lg">
            <span className="text-sm text-dark-400">Battery Voltage</span>
            <span className="text-sm text-primary-400 font-mono">{robotStatus.battery.voltage.toFixed(2)}V</span>
          </div>
          <div className="flex justify-between items-center p-2 bg-dark-800 rounded-lg">
            <span className="text-sm text-dark-400">Battery Current</span>
            <span className="text-sm text-primary-400 font-mono">{robotStatus.battery.current.toFixed(2)}A</span>
          </div>
          <div className="flex justify-between items-center p-2 bg-dark-800 rounded-lg">
            <span className="text-sm text-dark-400">Charging Status</span>
            <span className={`text-sm font-bold ${robotStatus.battery.charging ? 'text-green-500' : 'text-dark-400'}`}>
              {robotStatus.battery.charging ? 'Charging' : 'Not Charging'}
            </span>
          </div>
        </div>
      </div>

      {/* Robot Configuration */}
      <div className="card">
        <h3 className="card-header">
          <Zap className="w-4 h-4 inline mr-2" />
          Robot Hardware
        </h3>
        <div className="grid grid-cols-2 gap-3">
          <div className="p-2 bg-dark-800 rounded-lg">
            <p className="text-xs text-dark-400 mb-1">LIDAR</p>
            <p className="text-sm font-bold text-dark-200">SLLIDAR A1M8</p>
            <p className="text-xs text-dark-500 mt-0.5">360° laser scanner</p>
          </div>
          <div className="p-2 bg-dark-800 rounded-lg">
            <p className="text-xs text-dark-400 mb-1">Ultrasonic Sensors</p>
            <p className="text-sm font-bold text-dark-200">5 HC-SR04</p>
            <p className="text-xs text-dark-500 mt-0.5">Distance measurement</p>
          </div>
          <div className="p-2 bg-dark-800 rounded-lg">
            <p className="text-xs text-dark-400 mb-1">IR Sensors</p>
            <p className="text-sm font-bold text-dark-200">8 Sensors</p>
            <p className="text-xs text-dark-500 mt-0.5">4 obstacle + 4 cliff</p>
          </div>
          <div className="p-2 bg-dark-800 rounded-lg">
            <p className="text-xs text-dark-400 mb-1">Controller</p>
            <p className="text-sm font-bold text-dark-200">Raspberry Pi 4</p>
            <p className="text-xs text-dark-500 mt-0.5">ROS2 Humble</p>
          </div>
        </div>
      </div>

      {/* About */}
      <div className="card">
        <h3 className="card-header">About</h3>
        <div className="space-y-1 text-sm text-dark-300">
          <p><span className="text-dark-500">Dashboard Version:</span> 1.0.0</p>
          <p><span className="text-dark-500">Framework:</span> React 18 + TypeScript</p>
          <p><span className="text-dark-500">ROS Version:</span> ROS2 Humble</p>
          <p><span className="text-dark-500">SLAM:</span> SLAM Toolbox 2.6.10</p>
          <p><span className="text-dark-500">Algorithm:</span> Wall-Following Coverage Cleaner</p>
        </div>
      </div>
    </div>
  );
}
