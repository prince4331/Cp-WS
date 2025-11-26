import { LucideIcon } from 'lucide-react';

interface MetricCardProps {
  label: string;
  value: string | number;
  icon: LucideIcon;
  unit?: string;
}

export default function MetricCard({ label, value, icon: Icon, unit }: MetricCardProps) {
  return (
    <div className="card flex items-center gap-3">
      <Icon className="w-8 h-8 text-primary-500" />
      <div>
        <p className="text-xs text-dark-400">{label}</p>
        <p className="text-xl font-bold text-dark-50">
          {value}
          {unit && <span className="text-sm text-dark-400 ml-1">{unit}</span>}
        </p>
      </div>
    </div>
  );
}
