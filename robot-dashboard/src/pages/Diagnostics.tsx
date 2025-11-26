import { useRobotStore } from '../store/robotStore';
import SensorBar from '../components/widgets/SensorBar';
import { Activity, Wifi, Zap, AlertTriangle } from 'lucide-react';

export default function Diagnostics() {
  const robotStatus = useRobotStore((state) => state.robotStatus);
  const connected = useRobotStore((state) => state.connected);
  const sensors = robotStatus.sensors;

  const getHealthStatus = () => {
    if (!connected) return { status: 'Offline', color: 'text-red-500' };
    if (robotStatus.emergencyStop) return { status: 'Emergency Stop', color: 'text-red-500' };
    if (robotStatus.battery.percentage < 20) return { status: 'Low Battery', color: 'text-yellow-500' };
    return { status: 'Healthy', color: 'text-green-500' };
  };

  const health = getHealthStatus();

  return (
    <div className="space-y-3">
      {/* Header */}
      <div>
        <h2 className="text-2xl font-bold text-dark-50">System Diagnostics</h2>
        <p className="text-dark-400 text-sm">Monitor robot health and sensors</p>
      </div>

      {/* System Health */}
      <div className="grid grid-cols-4 gap-3">
        <div className="card">
          <div className="flex items-center gap-3">
            <Activity className={`w-6 h-6 ${health.color}`} />
            <div>
              <p className="text-xs text-dark-400">System Status</p>
              <p className={`text-sm font-bold ${health.color}`}>{health.status}</p>
            </div>
          </div>
        </div>
        <div className="card">
          <div className="flex items-center gap-3">
            <Wifi className={`w-6 h-6 ${connected ? 'text-green-500' : 'text-red-500'}`} />
            <div>
              <p className="text-xs text-dark-400">Connection</p>
              <p className={`text-sm font-bold ${connected ? 'text-green-500' : 'text-red-500'}`}>
                {connected ? 'Connected' : 'Disconnected'}
              </p>
            </div>
          </div>
        </div>
        <div className="card">
          <div className="flex items-center gap-3">
            <Zap className={`w-6 h-6 ${(robotStatus.battery.percentage || 0) > 20 ? 'text-green-500' : 'text-red-500'}`} />
            <div>
              <p className="text-xs text-dark-400">Battery</p>
              <p className={`text-sm font-bold ${(robotStatus.battery.percentage || 0) > 20 ? 'text-green-500' : 'text-red-500'}`}>
                {(robotStatus.battery.percentage || 0).toFixed(0)}%
              </p>
            </div>
          </div>
        </div>
        <div className="card">
          <div className="flex items-center gap-3">
            <AlertTriangle className={`w-6 h-6 ${robotStatus.emergencyStop ? 'text-red-500' : 'text-gray-500'}`} />
            <div>
              <p className="text-xs text-dark-400">E-Stop</p>
              <p className={`text-sm font-bold ${robotStatus.emergencyStop ? 'text-red-500' : 'text-gray-500'}`}>
                {robotStatus.emergencyStop ? 'ACTIVE' : 'Inactive'}
              </p>
            </div>
          </div>
        </div>
      </div>

      {/* Ultrasonic Sensors */}
      <div className="card">
        <h3 className="card-header">Ultrasonic Sensors (Distance)</h3>
        <div className="grid grid-cols-2 gap-4">
          <div className="space-y-2">
            <SensorBar label="Front Sensor" value={sensors.ultrasonic.front} />
            <SensorBar label="Left Corner" value={sensors.ultrasonic.left_corner} />
            <SensorBar label="Right Corner" value={sensors.ultrasonic.right_corner} />
          </div>
          <div className="space-y-2">
            <SensorBar label="Left Side" value={sensors.ultrasonic.left_side} />
            <SensorBar label="Right Side" value={sensors.ultrasonic.right_side} />
          </div>
        </div>
      </div>

      {/* IR Sensors */}
      <div className="grid grid-cols-2 gap-4">
        <div className="card">
          <h3 className="card-header">Obstacle Detection (IR)</h3>
          <div className="grid grid-cols-2 gap-2">
            <div className={`p-2 rounded-lg text-center ${sensors.ir.obstacle_front_left ? 'bg-red-500/20 border-2 border-red-500' : 'bg-dark-800'}`}>
              <p className="text-xs text-dark-400 mb-1">Front Left</p>
              <p className={`text-sm font-bold ${sensors.ir.obstacle_front_left ? 'text-red-500' : 'text-green-500'}`}>
                {sensors.ir.obstacle_front_left ? 'BLOCKED' : 'Clear'}
              </p>
            </div>
            <div className={`p-2 rounded-lg text-center ${sensors.ir.obstacle_front_right ? 'bg-red-500/20 border-2 border-red-500' : 'bg-dark-800'}`}>
              <p className="text-xs text-dark-400 mb-1">Front Right</p>
              <p className={`text-sm font-bold ${sensors.ir.obstacle_front_right ? 'text-red-500' : 'text-green-500'}`}>
                {sensors.ir.obstacle_front_right ? 'BLOCKED' : 'Clear'}
              </p>
            </div>
            <div className={`p-2 rounded-lg text-center ${sensors.ir.obstacle_back_left ? 'bg-red-500/20 border-2 border-red-500' : 'bg-dark-800'}`}>
              <p className="text-xs text-dark-400 mb-1">Back Left</p>
              <p className={`text-sm font-bold ${sensors.ir.obstacle_back_left ? 'text-red-500' : 'text-green-500'}`}>
                {sensors.ir.obstacle_back_left ? 'BLOCKED' : 'Clear'}
              </p>
            </div>
            <div className={`p-2 rounded-lg text-center ${sensors.ir.obstacle_back_right ? 'bg-red-500/20 border-2 border-red-500' : 'bg-dark-800'}`}>
              <p className="text-xs text-dark-400 mb-1">Back Right</p>
              <p className={`text-sm font-bold ${sensors.ir.obstacle_back_right ? 'text-red-500' : 'text-green-500'}`}>
                {sensors.ir.obstacle_back_right ? 'BLOCKED' : 'Clear'}
              </p>
            </div>
          </div>
        </div>

        <div className="card">
          <h3 className="card-header">Cliff Detection (IR)</h3>
          <div className="grid grid-cols-2 gap-2">
            <div className={`p-2 rounded-lg text-center ${sensors.ir.cliff_front_left ? 'bg-yellow-500/20 border-2 border-yellow-500' : 'bg-dark-800'}`}>
              <p className="text-xs text-dark-400 mb-1">Front Left</p>
              <p className={`text-sm font-bold ${sensors.ir.cliff_front_left ? 'text-yellow-500' : 'text-green-500'}`}>
                {sensors.ir.cliff_front_left ? 'CLIFF!' : 'Safe'}
              </p>
            </div>
            <div className={`p-2 rounded-lg text-center ${sensors.ir.cliff_front_right ? 'bg-yellow-500/20 border-2 border-yellow-500' : 'bg-dark-800'}`}>
              <p className="text-xs text-dark-400 mb-1">Front Right</p>
              <p className={`text-sm font-bold ${sensors.ir.cliff_front_right ? 'text-yellow-500' : 'text-green-500'}`}>
                {sensors.ir.cliff_front_right ? 'CLIFF!' : 'Safe'}
              </p>
            </div>
            <div className={`p-2 rounded-lg text-center ${sensors.ir.cliff_back_left ? 'bg-yellow-500/20 border-2 border-yellow-500' : 'bg-dark-800'}`}>
              <p className="text-xs text-dark-400 mb-1">Back Left</p>
              <p className={`text-sm font-bold ${sensors.ir.cliff_back_left ? 'text-yellow-500' : 'text-green-500'}`}>
                {sensors.ir.cliff_back_left ? 'CLIFF!' : 'Safe'}
              </p>
            </div>
            <div className={`p-2 rounded-lg text-center ${sensors.ir.cliff_back_right ? 'bg-yellow-500/20 border-2 border-yellow-500' : 'bg-dark-800'}`}>
              <p className="text-xs text-dark-400 mb-1">Back Right</p>
              <p className={`text-sm font-bold ${sensors.ir.cliff_back_right ? 'text-yellow-500' : 'text-green-500'}`}>
                {sensors.ir.cliff_back_right ? 'CLIFF!' : 'Safe'}
              </p>
            </div>
          </div>
        </div>
      </div>

      {/* IMU Data */}
      <div className="card">
        <h3 className="card-header">IMU (Inertial Measurement Unit)</h3>
        <div className="grid grid-cols-4 gap-3">
          <div>
            <p className="text-xs text-dark-400 mb-1">Heading</p>
            <p className="text-lg font-bold text-primary-400">
              {((robotStatus.pose.theta || 0) * 180 / Math.PI).toFixed(1)}°
            </p>
          </div>
          <div>
            <p className="text-xs text-dark-400 mb-1">Angular Velocity</p>
            <div className="space-y-0.5 text-xs">
              <p><span className="text-dark-500">X:</span> <span className="text-primary-400 font-mono">{(sensors.imu.angular_velocity.x || 0).toFixed(2)}</span></p>
              <p><span className="text-dark-500">Y:</span> <span className="text-primary-400 font-mono">{(sensors.imu.angular_velocity.y || 0).toFixed(2)}</span></p>
              <p><span className="text-dark-500">Z:</span> <span className="text-primary-400 font-mono">{(sensors.imu.angular_velocity.z || 0).toFixed(2)}</span></p>
            </div>
          </div>
          <div>
            <p className="text-xs text-dark-400 mb-1">Linear Acceleration</p>
            <div className="space-y-0.5 text-xs">
              <p><span className="text-dark-500">X:</span> <span className="text-primary-400 font-mono">{(sensors.imu.linear_acceleration.x || 0).toFixed(2)}</span></p>
              <p><span className="text-dark-500">Y:</span> <span className="text-primary-400 font-mono">{(sensors.imu.linear_acceleration.y || 0).toFixed(2)}</span></p>
              <p><span className="text-dark-500">Z:</span> <span className="text-primary-400 font-mono">{(sensors.imu.linear_acceleration.z || 0).toFixed(2)}</span></p>
            </div>
          </div>
          <div>
            <p className="text-xs text-dark-400 mb-1">Orientation (Quat)</p>
            <div className="space-y-0.5 text-xs">
              <p><span className="text-dark-500">X:</span> <span className="text-primary-400 font-mono">{(sensors.imu.orientation.x || 0).toFixed(2)}</span></p>
              <p><span className="text-dark-500">Y:</span> <span className="text-primary-400 font-mono">{(sensors.imu.orientation.y || 0).toFixed(2)}</span></p>
              <p><span className="text-dark-500">Z:</span> <span className="text-primary-400 font-mono">{(sensors.imu.orientation.z || 0).toFixed(2)}</span></p>
              <p><span className="text-dark-500">W:</span> <span className="text-primary-400 font-mono">{(sensors.imu.orientation.w || 1).toFixed(2)}</span></p>
            </div>
          </div>
        </div>
      </div>

      {/* LIDAR Status */}
      <div className="card">
        <h3 className="card-header">LIDAR Scanner</h3>
        <div className="grid grid-cols-4 gap-3">
          <div>
            <p className="text-xs text-dark-400">Scan Points</p>
            <p className="text-lg font-bold text-primary-400">{sensors.lidar.ranges.length}</p>
          </div>
          <div>
            <p className="text-xs text-dark-400">Angle Range</p>
            <p className="text-sm font-bold text-dark-200">
              {((sensors.lidar.angle_min || 0) * 180 / Math.PI).toFixed(0)}° to {((sensors.lidar.angle_max || 0) * 180 / Math.PI).toFixed(0)}°
            </p>
          </div>
          <div>
            <p className="text-xs text-dark-400">Resolution</p>
            <p className="text-sm font-bold text-dark-200">
              {((sensors.lidar.angle_increment || 0) * 180 / Math.PI).toFixed(2)}°
            </p>
          </div>
          <div>
            <p className="text-xs text-dark-400">Status</p>
            <p className="text-sm font-bold text-green-500">
              {sensors.lidar.ranges.length > 0 ? 'Active' : 'No Data'}
            </p>
          </div>
        </div>
      </div>
    </div>
  );
}
