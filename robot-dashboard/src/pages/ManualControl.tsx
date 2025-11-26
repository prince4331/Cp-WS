import { useRobotStore } from '../store/robotStore';
import Joystick from '../components/widgets/Joystick';
import SensorBar from '../components/widgets/SensorBar';
import { publishVelocity, publishRelayCommand, publishEmergencyStop } from '../lib/rosbridge';
import { ArrowUp, ArrowDown, ArrowLeft, ArrowRight, Square, Power } from 'lucide-react';
import { useEffect, useState } from 'react';

export default function ManualControl() {
  const robotStatus = useRobotStore((state) => state.robotStatus);
  const sensors = robotStatus.sensors;
  const maxLinearSpeed = useRobotStore((state) => state.maxLinearSpeed);
  const maxAngularSpeed = useRobotStore((state) => state.maxAngularSpeed);
  const setMaxLinearSpeed = useRobotStore((state) => state.setMaxLinearSpeed);
  const setMaxAngularSpeed = useRobotStore((state) => state.setMaxAngularSpeed);
  
  const cleaningSystems = useRobotStore((state) => state.cleaningSystems);
  const updateCleaningSystem = useRobotStore((state) => state.updateCleaningSystem);
  
  const [speedMultiplier, setSpeedMultiplier] = useState(0.5);

  // Handle cleaning system toggles
  const handleCleaningSystemToggle = (system: 'vacuumPump' | 'scrubber' | 'sweepingBrush' | 'waterPump', relay: number) => {
    const newState = !cleaningSystems[system];
    updateCleaningSystem(system, newState);
    publishRelayCommand(relay, newState);
  };

  // Keyboard control
  const [activeKeys, setActiveKeys] = useState<Set<string>>(new Set());

  // Continuous velocity publishing based on active keys
  useEffect(() => {
    const interval = setInterval(() => {
      let linear = 0;
      let angular = 0;

      if (activeKeys.has('ArrowUp') || activeKeys.has('w') || activeKeys.has('W')) {
        linear = maxLinearSpeed * speedMultiplier;
      }
      if (activeKeys.has('ArrowDown') || activeKeys.has('s') || activeKeys.has('S')) {
        linear = -maxLinearSpeed * speedMultiplier;
      }
      if (activeKeys.has('ArrowLeft') || activeKeys.has('a') || activeKeys.has('A')) {
        angular = maxAngularSpeed * speedMultiplier;
      }
      if (activeKeys.has('ArrowRight') || activeKeys.has('d') || activeKeys.has('D')) {
        angular = -maxAngularSpeed * speedMultiplier;
      }

      // Only publish if keys are pressed
      if (activeKeys.size > 0) {
        publishVelocity(linear, angular);
      }
    }, 100); // 10Hz

    return () => clearInterval(interval);
  }, [activeKeys, maxLinearSpeed, maxAngularSpeed, speedMultiplier]);

  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      const key = e.key;
      if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'w', 'a', 's', 'd', 'W', 'A', 'S', 'D'].includes(key)) {
        e.preventDefault();
        setActiveKeys(prev => new Set(prev).add(key));
      }
      if (key === ' ') {
        e.preventDefault();
        setActiveKeys(new Set());
        publishVelocity(0, 0);
      }
    };

    const handleKeyUp = (e: KeyboardEvent) => {
      const key = e.key;
      if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'w', 'a', 's', 'd', 'W', 'A', 'S', 'D'].includes(key)) {
        setActiveKeys(prev => {
          const newSet = new Set(prev);
          newSet.delete(key);
          if (newSet.size === 0) {
            publishVelocity(0, 0);
          }
          return newSet;
        });
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);

    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, [maxLinearSpeed, maxAngularSpeed, speedMultiplier]);

  const handleStop = () => publishVelocity(0, 0);
  const handleEmergencyStop = () => {
    publishEmergencyStop(true);
    setTimeout(() => publishEmergencyStop(false), 200);
  };

  return (
    <div className="space-y-3">
      {/* Header */}
      <div>
        <h2 className="text-2xl font-bold text-dark-50">Manual Control</h2>
        <p className="text-dark-400 text-sm">Control the robot with joystick or keyboard</p>
      </div>

      <div className="grid grid-cols-1 xl:grid-cols-2 gap-4">
        {/* Left: Joystick Control */}
        <div className="space-y-3">
          <div className="card">
            <h3 className="card-header">Joystick Control</h3>
            <div className="flex justify-center py-3">
              <Joystick />
            </div>
          </div>

          {/* Speed Control */}
          <div className="card">
            <h3 className="card-header">Speed Settings</h3>
            <div className="space-y-3">
              <div>
                <label className="text-sm text-dark-400 mb-2 block">
                  Speed Multiplier: {(speedMultiplier * 100).toFixed(0)}%
                </label>
                <input
                  type="range"
                  min="0.1"
                  max="1.0"
                  step="0.1"
                  value={speedMultiplier}
                  onChange={(e) => setSpeedMultiplier(parseFloat(e.target.value))}
                  className="w-full"
                />
              </div>
              
              <div>
                <label className="text-sm text-dark-400 mb-2 block">
                  Max Linear Speed: {maxLinearSpeed.toFixed(2)} m/s
                </label>
                <input
                  type="range"
                  min="0.1"
                  max="0.5"
                  step="0.05"
                  value={maxLinearSpeed}
                  onChange={(e) => setMaxLinearSpeed(parseFloat(e.target.value))}
                  className="w-full"
                />
              </div>

              <div>
                <label className="text-sm text-dark-400 mb-2 block">
                  Max Angular Speed: {maxAngularSpeed.toFixed(2)} rad/s
                </label>
                <input
                  type="range"
                  min="0.5"
                  max="2.0"
                  step="0.1"
                  value={maxAngularSpeed}
                  onChange={(e) => setMaxAngularSpeed(parseFloat(e.target.value))}
                  className="w-full"
                />
              </div>
            </div>
          </div>

          {/* Cleaning Systems Control */}
          <div className="card">
            <h3 className="card-header">Cleaning Systems</h3>
            <div className="space-y-2">
              {/* Vacuum Pump */}
              <button
                onClick={() => handleCleaningSystemToggle('vacuumPump', 1)}
                className={`w-full p-3 rounded-lg flex items-center justify-between transition-colors ${
                  cleaningSystems.vacuumPump
                    ? 'bg-green-500/20 border-2 border-green-500 text-green-400'
                    : 'bg-dark-800 border-2 border-dark-700 text-dark-400 hover:border-dark-600'
                }`}
              >
                <span className="flex items-center gap-2">
                  <Power className="w-5 h-5" />
                  <span className="font-medium">Vacuum Pump</span>
                </span>
                <span className="text-sm font-bold">
                  {cleaningSystems.vacuumPump ? 'ON' : 'OFF'}
                </span>
              </button>

              {/* Scrubber */}
              <button
                onClick={() => handleCleaningSystemToggle('scrubber', 2)}
                className={`w-full p-3 rounded-lg flex items-center justify-between transition-colors ${
                  cleaningSystems.scrubber
                    ? 'bg-blue-500/20 border-2 border-blue-500 text-blue-400'
                    : 'bg-dark-800 border-2 border-dark-700 text-dark-400 hover:border-dark-600'
                }`}
              >
                <span className="flex items-center gap-2">
                  <Power className="w-5 h-5" />
                  <span className="font-medium">Scrubber</span>
                </span>
                <span className="text-sm font-bold">
                  {cleaningSystems.scrubber ? 'ON' : 'OFF'}
                </span>
              </button>

              {/* Sweeping Brush */}
              <button
                onClick={() => handleCleaningSystemToggle('sweepingBrush', 3)}
                className={`w-full p-3 rounded-lg flex items-center justify-between transition-colors ${
                  cleaningSystems.sweepingBrush
                    ? 'bg-yellow-500/20 border-2 border-yellow-500 text-yellow-400'
                    : 'bg-dark-800 border-2 border-dark-700 text-dark-400 hover:border-dark-600'
                }`}
              >
                <span className="flex items-center gap-2">
                  <Power className="w-5 h-5" />
                  <span className="font-medium">Sweeping Brush</span>
                </span>
                <span className="text-sm font-bold">
                  {cleaningSystems.sweepingBrush ? 'ON' : 'OFF'}
                </span>
              </button>

              {/* Water Pump */}
              <button
                onClick={() => handleCleaningSystemToggle('waterPump', 4)}
                className={`w-full p-3 rounded-lg flex items-center justify-between transition-colors ${
                  cleaningSystems.waterPump
                    ? 'bg-cyan-500/20 border-2 border-cyan-500 text-cyan-400'
                    : 'bg-dark-800 border-2 border-dark-700 text-dark-400 hover:border-dark-600'
                }`}
              >
                <span className="flex items-center gap-2">
                  <Power className="w-5 h-5" />
                  <span className="font-medium">Water Pump</span>
                </span>
                <span className="text-sm font-bold">
                  {cleaningSystems.waterPump ? 'ON' : 'OFF'}
                </span>
              </button>
            </div>
          </div>

          {/* Keyboard Hints */}
          <div className="card">
            <h3 className="card-header">Keyboard Shortcuts</h3>
            <div className="grid grid-cols-3 sm:grid-cols-6 gap-2 text-center">
              <div className="p-2 bg-dark-800 rounded-lg">
                <ArrowUp className="w-5 h-5 mx-auto mb-1 text-primary-400" />
                <p className="text-xs text-dark-400">Forward</p>
              </div>
              <div className="p-2 bg-dark-800 rounded-lg">
                <ArrowDown className="w-5 h-5 mx-auto mb-1 text-primary-400" />
                <p className="text-xs text-dark-400">Backward</p>
              </div>
              <div className="p-2 bg-dark-800 rounded-lg">
                <ArrowLeft className="w-5 h-5 mx-auto mb-1 text-primary-400" />
                <p className="text-xs text-dark-400">Left</p>
              </div>
              <div className="p-2 bg-dark-800 rounded-lg">
                <ArrowRight className="w-5 h-5 mx-auto mb-1 text-primary-400" />
                <p className="text-xs text-dark-400">Right</p>
              </div>
              <div className="p-2 bg-dark-800 rounded-lg">
                <Square className="w-5 h-5 mx-auto mb-1 text-red-400" />
                <p className="text-xs text-dark-400">Space</p>
              </div>
              <div className="p-2 bg-dark-800 rounded-lg">
                <p className="text-sm font-bold text-primary-400 mb-1">WASD</p>
                <p className="text-xs text-dark-400">Alt</p>
              </div>
            </div>
          </div>
        </div>

        {/* Right: Sensors */}
        <div className="space-y-3">
          {/* Current Velocity */}
          <div className="card">
            <h3 className="card-header">Current Velocity</h3>
            <div className="grid grid-cols-1 sm:grid-cols-2 gap-4">
              <div>
                <p className="text-sm text-dark-400">Linear</p>
                <p className="text-2xl font-bold text-primary-400">
                  {robotStatus.velocity.linear.toFixed(2)} <span className="text-sm">m/s</span>
                </p>
              </div>
              <div>
                <p className="text-sm text-dark-400">Angular</p>
                <p className="text-2xl font-bold text-primary-400">
                  {robotStatus.velocity.angular.toFixed(2)} <span className="text-sm">rad/s</span>
                </p>
              </div>
            </div>
            <div className="grid grid-cols-1 sm:grid-cols-2 gap-3 mt-2">
              <button
                onClick={handleStop}
                className="btn-danger w-full flex items-center justify-center gap-2"
                title="Publish zero velocity"
              >
                <Square className="w-5 h-5" />
                STOP MOTORS
              </button>
              <button
                onClick={handleEmergencyStop}
                className="btn-warning w-full flex items-center justify-center gap-2"
                title="Trigger emergency stop relay"
              >
                <Square className="w-5 h-5" />
                E-STOP
              </button>
            </div>
          </div>

          {/* Ultrasonic Sensors */}
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

          {/* IR Sensors */}
          <div className="card">
            <h3 className="card-header">IR Sensors</h3>
            <div className="grid grid-cols-1 sm:grid-cols-2 gap-2">
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
    </div>
  );
}
