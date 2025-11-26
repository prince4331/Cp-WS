import { ReactNode } from 'react';
import Sidebar from './Sidebar';
import Header from './Header';
import { useROS } from '../../hooks/useROS';

interface LayoutProps {
  children: ReactNode;
}

export default function Layout({ children }: LayoutProps) {
  // Initialize ROS connection
  useROS();

  return (
    <div className="flex h-screen bg-dark-950 text-dark-50">
      <Sidebar />
      <div className="flex-1 flex flex-col overflow-hidden">
        <Header />
        <main className="flex-1 overflow-auto p-3">
          {children}
        </main>
      </div>
    </div>
  );
}
