import { formatDistance } from '../../lib/utils';

interface SensorBarProps {
  label: string;
  value: number;
  maxValue?: number;
  warningThreshold?: number;
  dangerThreshold?: number;
}

export default function SensorBar({
  label,
  value,
  maxValue = 300, // 300cm = 3m (ultrasonic max range)
  warningThreshold = 50, // 50cm warning
  dangerThreshold = 25, // 25cm danger
}: SensorBarProps) {
  const percentage = Math.min((value / maxValue) * 100, 100);
  
  let color = 'bg-green-500';
  if (value < dangerThreshold) {
    color = 'bg-red-500';
  } else if (value < warningThreshold) {
    color = 'bg-yellow-500';
  }

  return (
    <div className="space-y-1">
      <div className="flex justify-between text-xs">
        <span className="text-dark-400">{label}</span>
        <span className="text-dark-200 font-medium">{formatDistance(value)}</span>
      </div>
      <div className="h-2 bg-dark-800 rounded-full overflow-hidden">
        <div
          className={`h-full ${color} transition-all duration-300`}
          style={{ width: `${percentage}%` }}
        />
      </div>
    </div>
  );
}
