import { useRobotStore } from '../../store/robotStore';
import { publishEmergencyStop } from '../../lib/rosbridge';
import { AlertOctagon } from 'lucide-react';

export default function EmergencyStop() {
  const emergencyStop = useRobotStore((state) => state.robotStatus.emergencyStop);
  const setEmergencyStop = useRobotStore((state) => state.setEmergencyStop);

  const handleClick = () => {
    const newState = !emergencyStop;
    setEmergencyStop(newState);
    publishEmergencyStop(newState);
  };

  return (
    <button
      onClick={handleClick}
      className={`flex items-center gap-2 px-4 py-2 rounded-lg font-bold transition-all ${
        emergencyStop
          ? 'bg-green-600 hover:bg-green-700 text-white'
          : 'bg-red-600 hover:bg-red-700 text-white shadow-glow-red'
      }`}
    >
      <AlertOctagon className="w-5 h-5" />
      <span>{emergencyStop ? 'RELEASE' : 'E-STOP'}</span>
    </button>
  );
}
