import { useRobotStore } from '../store/robotStore';
import { publishCleaningCommand } from '../lib/rosbridge';
import MetricCard from '../components/widgets/MetricCard';
import MapCanvas from '../components/widgets/MapCanvas';
import { Play, Square, Pause, Navigation2, Clock, Maximize, Route } from 'lucide-react';
import { formatTime } from '../lib/utils';

export default function AutoClean() {
  const robotStatus = useRobotStore((state) => state.robotStatus);
  const mission = robotStatus.mission;
  const setMissionState = useRobotStore((state) => state.setMissionState);
  const updateMode = useRobotStore((state) => state.updateMode);

  const isRunning = mission?.state !== 'IDLE' && mission?.state !== 'EMERGENCY';

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
        <h2 className="text-2xl font-bold text-dark-50">Autonomous Cleaning</h2>
        <p className="text-dark-400 text-sm">Wall-following coverage cleaner</p>
      </div>

      {/* Control Panel */}
      <div className="card">
        <h3 className="card-header">Mission Control</h3>
        <div className="flex flex-col sm:flex-row gap-3">
          <button
            onClick={handleStart}
            disabled={isRunning}
            className={`btn-primary flex-1 flex items-center justify-center gap-2 ${
              isRunning ? 'opacity-50 cursor-not-allowed' : ''
            }`}
          >
            <Play className="w-5 h-5" />
            Start Cleaning
          </button>
          <button
            onClick={handlePause}
            disabled={!isRunning}
            className={`btn-secondary flex-1 flex items-center justify-center gap-2 ${
              !isRunning ? 'opacity-50 cursor-not-allowed' : ''
            }`}
          >
            <Pause className="w-5 h-5" />
            Pause
          </button>
          <button
            onClick={handleStop}
            disabled={!isRunning}
            className={`btn-danger flex-1 flex items-center justify-center gap-2 ${
              !isRunning ? 'opacity-50 cursor-not-allowed' : ''
            }`}
          >
            <Square className="w-5 h-5" />
            Stop
          </button>
        </div>
      </div>

      {/* Status Display */}
      <div className="grid grid-cols-1 sm:grid-cols-2 xl:grid-cols-4 gap-3">
        <MetricCard
          label="Current State"
          value={mission?.state || 'IDLE'}
          icon={Navigation2}
        />
        <MetricCard
          label="Coverage Area"
          value={mission?.coverageArea.toFixed(1) || '0.0'}
          icon={Maximize}
          unit="m²"
        />
        <MetricCard
          label="Distance Traveled"
          value={mission?.distanceTraveled.toFixed(1) || '0.0'}
          icon={Route}
          unit="m"
        />
        <MetricCard
          label="Cleaning Time"
          value={formatTime(mission?.cleaningTime || 0)}
          icon={Clock}
        />
      </div>

      {/* Progress Bar */}
      <div className="card">
        <h3 className="card-header">Progress</h3>
        <div className="space-y-1">
          <div className="flex justify-between text-sm">
            <span className="text-dark-400">Overall Progress</span>
            <span className="text-primary-400 font-bold">{mission?.progress || 0}%</span>
          </div>
          <div className="h-3 bg-dark-800 rounded-full overflow-hidden">
            <div
              className="h-full bg-primary-500 transition-all duration-500"
              style={{ width: `${mission?.progress || 0}%` }}
            />
          </div>
        </div>
      </div>

      {/* Map View */}
      <div className="card">
        <h3 className="card-header">Live Coverage Map</h3>
        <div className="flex justify-center w-full">
          <MapCanvas width={700} height={500} showLidar={true} showRobot={true} />
        </div>
      </div>

      {/* Cleaning Algorithm Info */}
      <div className="card">
        <h3 className="card-header">Wall-Following Algorithm</h3>
        <div className="space-y-2 text-sm text-dark-300">
          <div className="flex items-start gap-2">
            <div className="w-6 h-6 rounded-full bg-primary-600 flex items-center justify-center text-white font-bold flex-shrink-0 text-xs">
              1
            </div>
            <div>
              <p className="font-semibold text-dark-100">Finding Wall</p>
              <p className="text-dark-400 text-xs">Robot rotates to detect nearby walls using LIDAR</p>
            </div>
          </div>
          <div className="flex items-start gap-2">
            <div className="w-6 h-6 rounded-full bg-primary-600 flex items-center justify-center text-white font-bold flex-shrink-0 text-xs">
              2
            </div>
            <div>
              <p className="font-semibold text-dark-100">Wall Following</p>
              <p className="text-dark-400 text-xs">Maintains 30cm distance from wall using PID control</p>
            </div>
          </div>
          <div className="flex items-start gap-2">
            <div className="w-6 h-6 rounded-full bg-primary-600 flex items-center justify-center text-white font-bold flex-shrink-0 text-xs">
              3
            </div>
            <div>
              <p className="font-semibold text-dark-100">Corner Turning</p>
              <p className="text-dark-400 text-xs">Executes 90° turns at corners to continue coverage</p>
            </div>
          </div>
          <div className="flex items-start gap-2">
            <div className="w-6 h-6 rounded-full bg-primary-600 flex items-center justify-center text-white font-bold flex-shrink-0 text-xs">
              4
            </div>
            <div>
              <p className="font-semibold text-dark-100">Spiral Inward</p>
              <p className="text-dark-400 text-xs">Reduces wall distance to cover interior areas</p>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
