import { useRobotStore } from '../../store/robotStore';
import { Wifi, WifiOff, AlertCircle } from 'lucide-react';
import BatteryIndicator from '../widgets/BatteryIndicator';
import EmergencyStop from '../widgets/EmergencyStop';

export default function Header() {
  const connected = useRobotStore((state) => state.connected);
  const robotStatus = useRobotStore((state) => state.robotStatus);
  const emergencyStop = robotStatus.emergencyStop;

  return (
    <header className="h-16 bg-dark-900 border-b border-dark-800 flex items-center justify-between px-6">
      {/* Left: Connection Status */}
      <div className="flex items-center gap-4">
        <div className="flex items-center gap-2">
          {connected ? (
            <>
              <Wifi className="w-5 h-5 text-green-500" />
              <span className="text-sm font-medium text-green-500">Connected</span>
            </>
          ) : (
            <>
              <WifiOff className="w-5 h-5 text-red-500" />
              <span className="text-sm font-medium text-red-500">Disconnected</span>
            </>
          )}
        </div>

        {emergencyStop && (
          <div className="flex items-center gap-2 px-3 py-1 bg-red-500/20 border border-red-500 rounded-lg">
            <AlertCircle className="w-4 h-4 text-red-500" />
            <span className="text-sm font-medium text-red-500">EMERGENCY STOP</span>
          </div>
        )}
      </div>

      {/* Center: Title */}
      <div className="flex-1 text-center">
        <h1 className="text-xl font-bold text-dark-50">Floor Cleaning Robot</h1>
        <p className="text-xs text-dark-400">
          Mode: <span className="text-primary-400 font-medium">{robotStatus.mode}</span>
        </p>
      </div>

      {/* Right: Battery & Emergency Stop */}
      <div className="flex items-center gap-4">
        <BatteryIndicator />
        <EmergencyStop />
      </div>
    </header>
  );
}
