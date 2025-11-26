import { useRobotStore } from '../store/robotStore';
import MapCanvas from '../components/widgets/MapCanvas';
import LidarVisualizer from '../components/widgets/LidarVisualizer';
import StatusCard from '../components/widgets/StatusCard';
import SensorBar from '../components/widgets/SensorBar';
import { publishCleaningCommand } from '../lib/rosbridge';
import { 
  Play, 
  Pause, 
  Square, 
  Navigation, 
  Clock, 
  Maximize,
  Gauge,
  Power
} from 'lucide-react';
import { formatTime } from '../lib/utils';

export default function HomePage() {
  const robotStatus = useRobotStore((state) => state.robotStatus);
  const sensors = robotStatus.sensors;
  const mission = robotStatus.mission;
  const cleaningSystems = useRobotStore((state) => state.cleaningSystems);
  const setMissionState = useRobotStore((state) => state.setMissionState);
  const updateMode = useRobotStore((state) => state.updateMode);

  const handleStart = () => {
    setMissionState('FINDING_WALL');
    updateMode('AUTO');
    publishCleaningCommand('start');
  };
  const handlePause = () => {
    setMissionState('IDLE');
    updateMode('IDLE');
    publishCleaningCommand('pause');
  };
  const handleStop = () => {
    setMissionState('IDLE');
    updateMode('MANUAL');
    publishCleaningCommand('stop');
  };

  return (
    <div className="space-y-3">
      {/* Header */}
      <div>
        <h2 className="text-2xl font-bold text-dark-50">Robot Dashboard</h2>
        <p className="text-dark-400 text-sm">Monitor and control your cleaning robot</p>
      </div>

      {/* Status Cards Row */}
      <div className="grid grid-cols-1 sm:grid-cols-2 xl:grid-cols-4 gap-3">
        <StatusCard
          title="Current State"
          value={mission?.state || 'IDLE'}
          icon={Navigation}
          color={mission?.state === 'EMERGENCY' ? 'red' : mission?.state === 'IDLE' ? 'gray' : 'blue'}
        />
        <StatusCard
          title="Progress"
          value={`${mission?.progress || 0}%`}
          icon={Gauge}
          color="green"
        />
        <StatusCard
          title="Coverage Area"
          value={mission?.coverageArea.toFixed(1) || '0.0'}
          icon={Maximize}
          color="blue"
          subtitle="square meters"
        />
        <StatusCard
          title="Cleaning Time"
          value={formatTime(mission?.cleaningTime || 0)}
          icon={Clock}
          color="yellow"
        />
      </div>

      {/* Main Content Grid */}
      <div className="grid grid-cols-1 lg:grid-cols-3 gap-4">
        {/* Left column: Map + sensors */}
        <div className="space-y-3 lg:col-span-2">
          <div className="card">
            <h3 className="card-header">Live Map</h3>
            <div className="flex justify-center w-full">
              <MapCanvas width={450} height={450} showLidar={true} showRobot={true} />
            </div>
          </div>

          <div className="grid grid-cols-1 md:grid-cols-2 gap-3">
            <div className="card">
              <h3 className="card-header">Ultrasonic Sensors</h3>
              <div className="space-y-2">
                <SensorBar label="Front" value={sensors.ultrasonic.front} />
                <SensorBar label="Left Corner" value={sensors.ultrasonic.left_corner} />
                <SensorBar label="Right Corner" value={sensors.ultrasonic.right_corner} />
                <SensorBar label="Left Side" value={sensors.ultrasonic.left_side} />
                <SensorBar label="Right Side" value={sensors.ultrasonic.right_side} />
              </div>
            </div>

            <div className="card">
              <h3 className="card-header">IR Sensors</h3>
              <div className="grid grid-cols-2 gap-2">
                <div className={`p-2 rounded-lg ${sensors.ir.obstacle_front_left ? 'bg-red-500/20 border border-red-500' : 'bg-dark-800'}`}>
                  <p className="text-xs text-dark-400">Front Left</p>
                  <p className="text-sm font-bold">{sensors.ir.obstacle_front_left ? 'BLOCKED' : 'Clear'}</p>
                </div>
                <div className={`p-2 rounded-lg ${sensors.ir.obstacle_front_right ? 'bg-red-500/20 border border-red-500' : 'bg-dark-800'}`}>
                  <p className="text-xs text-dark-400">Front Right</p>
                  <p className="text-sm font-bold">{sensors.ir.obstacle_front_right ? 'BLOCKED' : 'Clear'}</p>
                </div>
                <div className={`p-2 rounded-lg ${sensors.ir.cliff_front_left ? 'bg-yellow-500/20 border border-yellow-500' : 'bg-dark-800'}`}>
                  <p className="text-xs text-dark-400">Cliff FL</p>
                  <p className="text-sm font-bold">{sensors.ir.cliff_front_left ? 'DETECTED' : 'OK'}</p>
                </div>
                <div className={`p-2 rounded-lg ${sensors.ir.cliff_front_right ? 'bg-yellow-500/20 border border-yellow-500' : 'bg-dark-800'}`}>
                  <p className="text-xs text-dark-400">Cliff FR</p>
                  <p className="text-sm font-bold">{sensors.ir.cliff_front_right ? 'DETECTED' : 'OK'}</p>
                </div>
              </div>
            </div>
          </div>
        </div>

        {/* Right column: controls + status */}
        <div className="space-y-3">
          <div className="card">
            <h3 className="card-header">Mission Control</h3>
            <div className="flex flex-col sm:flex-row gap-3">
              <button
                onClick={handleStart}
                disabled={mission?.state !== 'IDLE' && mission?.state !== 'EMERGENCY'}
                className="btn-primary flex-1 flex items-center justify-center gap-2"
              >
                <Play className="w-5 h-5" />
                Start
              </button>
              <button
                onClick={handlePause}
                disabled={mission?.state === 'IDLE'}
                className="btn-secondary flex-1 flex items-center justify-center gap-2"
              >
                <Pause className="w-5 h-5" />
                Pause
              </button>
              <button
                onClick={handleStop}
                disabled={mission?.state === 'IDLE'}
                className="btn-danger flex-1 flex items-center justify-center gap-2"
              >
                <Square className="w-5 h-5" />
                Stop
              </button>
            </div>
          </div>

          <div className="card">
            <h3 className="card-header">Cleaning Systems Status</h3>
            <div className="grid grid-cols-1 sm:grid-cols-2 gap-2">
              <div className={`p-3 rounded-lg flex items-center gap-2 ${
                cleaningSystems.vacuumPump
                  ? 'bg-green-500/20 border border-green-500'
                  : 'bg-dark-800 border border-dark-700'
              }`}>
                <Power className={`w-4 h-4 ${cleaningSystems.vacuumPump ? 'text-green-400' : 'text-dark-500'}`} />
                <div className="flex-1">
                  <p className="text-xs text-dark-400">Vacuum Pump</p>
                  <p className={`text-sm font-bold ${cleaningSystems.vacuumPump ? 'text-green-400' : 'text-dark-500'}`}>
                    {cleaningSystems.vacuumPump ? 'ON' : 'OFF'}
                  </p>
                </div>
              </div>
              <div className={`p-3 rounded-lg flex items-center gap-2 ${
                cleaningSystems.scrubber
                  ? 'bg-blue-500/20 border border-blue-500'
                  : 'bg-dark-800 border border-dark-700'
              }`}>
                <Power className={`w-4 h-4 ${cleaningSystems.scrubber ? 'text-blue-400' : 'text-dark-500'}`} />
                <div className="flex-1">
                  <p className="text-xs text-dark-400">Scrubber</p>
                  <p className={`text-sm font-bold ${cleaningSystems.scrubber ? 'text-blue-400' : 'text-dark-500'}`}>
                    {cleaningSystems.scrubber ? 'ON' : 'OFF'}
                  </p>
                </div>
              </div>
              <div className={`p-3 rounded-lg flex items-center gap-2 ${
                cleaningSystems.sweepingBrush
                  ? 'bg-yellow-500/20 border border-yellow-500'
                  : 'bg-dark-800 border border-dark-700'
              }`}>
                <Power className={`w-4 h-4 ${cleaningSystems.sweepingBrush ? 'text-yellow-400' : 'text-dark-500'}`} />
                <div className="flex-1">
                  <p className="text-xs text-dark-400">Sweeping Brush</p>
                  <p className={`text-sm font-bold ${cleaningSystems.sweepingBrush ? 'text-yellow-400' : 'text-dark-500'}`}>
                    {cleaningSystems.sweepingBrush ? 'ON' : 'OFF'}
                  </p>
                </div>
              </div>
              <div className={`p-3 rounded-lg flex items-center gap-2 ${
                cleaningSystems.waterPump
                  ? 'bg-cyan-500/20 border border-cyan-500'
                  : 'bg-dark-800 border border-dark-700'
              }`}>
                <Power className={`w-4 h-4 ${cleaningSystems.waterPump ? 'text-cyan-400' : 'text-dark-500'}`} />
                <div className="flex-1">
                  <p className="text-xs text-dark-400">Water Pump</p>
                  <p className={`text-sm font-bold ${cleaningSystems.waterPump ? 'text-cyan-400' : 'text-dark-500'}`}>
                    {cleaningSystems.waterPump ? 'ON' : 'OFF'}
                  </p>
                </div>
              </div>
            </div>
          </div>

          <div className="card">
            <h3 className="card-header">LIDAR Scan</h3>
            <div className="flex justify-center w-full">
              <LidarVisualizer size={240} />
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
