import { ReactNode } from 'react';
import { LucideIcon } from 'lucide-react';

interface StatusCardProps {
  title: string;
  value: string | number;
  icon: LucideIcon;
  color?: 'blue' | 'green' | 'yellow' | 'red' | 'gray';
  subtitle?: string;
  children?: ReactNode;
}

const colorClasses = {
  blue: 'text-primary-500 bg-primary-500/20',
  green: 'text-green-500 bg-green-500/20',
  yellow: 'text-yellow-500 bg-yellow-500/20',
  red: 'text-red-500 bg-red-500/20',
  gray: 'text-dark-400 bg-dark-800',
};

export default function StatusCard({
  title,
  value,
  icon: Icon,
  color = 'blue',
  subtitle,
  children,
}: StatusCardProps) {
  return (
    <div className="card">
      <div className="flex items-start gap-3">
        <div className={`p-3 rounded-lg ${colorClasses[color]}`}>
          <Icon className="w-6 h-6" />
        </div>
        <div className="flex-1">
          <p className="text-sm text-dark-400">{title}</p>
          <p className="text-2xl font-bold text-dark-50 mt-1">{value}</p>
          {subtitle && <p className="text-xs text-dark-500 mt-1">{subtitle}</p>}
          {children && <div className="mt-3">{children}</div>}
        </div>
      </div>
    </div>
  );
}
