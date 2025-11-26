import { useRobotStore } from '../../store/robotStore';
import { Battery, BatteryCharging, BatteryWarning } from 'lucide-react';
import { formatBattery, getBatteryColor } from '../../lib/utils';

export default function BatteryIndicator() {
  const battery = useRobotStore((state) => state.robotStatus.battery);

  const Icon = battery.charging
    ? BatteryCharging
    : battery.percentage < 20
    ? BatteryWarning
    : Battery;

  return (
    <div className="flex items-center gap-2 px-3 py-1.5 bg-dark-800 rounded-lg">
      <Icon className={`w-5 h-5 ${getBatteryColor(battery.percentage)}`} />
      <div className="flex flex-col">
        <span className={`text-sm font-bold ${getBatteryColor(battery.percentage)}`}>
          {formatBattery(battery.percentage)}
        </span>
        <div className="flex gap-2 text-xs text-dark-400">
          <span>{battery.voltage.toFixed(1)}V</span>
          <span>•</span>
          <span>{battery.current.toFixed(2)}A</span>
        </div>
      </div>
    </div>
  );
}
