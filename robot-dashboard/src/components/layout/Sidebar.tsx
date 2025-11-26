import { NavLink } from 'react-router-dom';
import { 
  Home, 
  Gamepad2, 
  Play, 
  Map, 
  Activity, 
  Settings,
  Bot
} from 'lucide-react';

const navItems = [
  { path: '/', icon: Home, label: 'Home' },
  { path: '/manual', icon: Gamepad2, label: 'Manual' },
  { path: '/auto', icon: Play, label: 'Auto Clean' },
  { path: '/map', icon: Map, label: 'Map' },
  { path: '/diagnostics', icon: Activity, label: 'Diagnostics' },
  { path: '/settings', icon: Settings, label: 'Settings' },
];

export default function Sidebar() {
  return (
    <aside className="w-20 bg-dark-900 border-r border-dark-800 flex flex-col items-center py-6">
      {/* Logo */}
      <div className="mb-8 p-3 bg-primary-600 rounded-lg">
        <Bot className="w-8 h-8 text-white" />
      </div>

      {/* Navigation */}
      <nav className="flex-1 flex flex-col gap-4">
        {navItems.map((item) => {
          const Icon = item.icon;
          return (
            <NavLink
              key={item.path}
              to={item.path}
              className={({ isActive }) =>
                `flex flex-col items-center gap-1 p-3 rounded-lg transition-colors ${
                  isActive
                    ? 'bg-primary-600 text-white'
                    : 'text-dark-400 hover:text-dark-200 hover:bg-dark-800'
                }`
              }
              title={item.label}
            >
              <Icon className="w-6 h-6" />
              <span className="text-xs">{item.label.split(' ')[0]}</span>
            </NavLink>
          );
        })}
      </nav>
    </aside>
  );
}
