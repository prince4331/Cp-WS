import MapCanvas from '../components/widgets/MapCanvas';
import LidarVisualizer from '../components/widgets/LidarVisualizer';
import { useRobotStore } from '../store/robotStore';

export default function MapPage() {
  const robotStatus = useRobotStore((state) => state.robotStatus);
  const mapData = useRobotStore((state) => state.mapData);

  return (
    <div className="space-y-3">
      {/* Header */}
      <div>
        <h2 className="text-2xl font-bold text-dark-50">Map Visualization</h2>
        <p className="text-dark-400 text-sm">Real-time SLAM mapping</p>
      </div>

      {/* Map Info */}
      {mapData && (
        <div className="grid grid-cols-4 gap-3">
          <div className="card">
            <p className="text-xs text-dark-400">Map Size</p>
            <p className="text-lg font-bold text-dark-50">
              {mapData.width} × {mapData.height}
            </p>
          </div>
          <div className="card">
            <p className="text-xs text-dark-400">Resolution</p>
            <p className="text-lg font-bold text-dark-50">
              {mapData.resolution.toFixed(3)} m/px
            </p>
          </div>
          <div className="card">
            <p className="text-xs text-dark-400">Robot Position</p>
            <p className="text-lg font-bold text-dark-50">
              ({robotStatus.pose.x.toFixed(2)}, {robotStatus.pose.y.toFixed(2)})
            </p>
          </div>
          <div className="card">
            <p className="text-xs text-dark-400">Heading</p>
            <p className="text-lg font-bold text-dark-50">
              {(robotStatus.pose.theta * 180 / Math.PI).toFixed(1)}°
            </p>
          </div>
        </div>
      )}

      {/* Main Map */}
      <div className="grid grid-cols-3 gap-4">
        <div className="col-span-3 card">
          <h3 className="card-header">SLAM Map</h3>
          <div className="flex justify-center">
            <MapCanvas width={1000} height={600} showLidar={true} showRobot={true} />
          </div>
        </div>
      </div>

      {/* Side panels below map */}
      <div className="grid grid-cols-3 gap-4">
        <div className="space-y-3">
          {/* LIDAR View */}
          <div className="card">
            <h3 className="card-header">LIDAR Scan</h3>
            <div className="flex justify-center">
              <LidarVisualizer size={260} />
            </div>
          </div>

          {/* Legend */}
          <div className="card">
            <h3 className="card-header">Legend</h3>
            <div className="space-y-1 text-sm">
              <div className="flex items-center gap-2">
                <div className="w-4 h-4 bg-dark-900 border border-dark-700" />
                <span className="text-dark-300">Free Space</span>
              </div>
              <div className="flex items-center gap-2">
                <div className="w-4 h-4 bg-dark-600" />
                <span className="text-dark-300">Occupied</span>
              </div>
              <div className="flex items-center gap-2">
                <div className="w-4 h-4 bg-dark-800" />
                <span className="text-dark-300">Unknown</span>
              </div>
              <div className="flex items-center gap-2">
                <div className="w-4 h-4 bg-primary-500 rounded-full" />
                <span className="text-dark-300">Robot Position</span>
              </div>
              <div className="flex items-center gap-2">
                <div className="w-4 h-4 border-2 border-primary-400" />
                <span className="text-dark-300">LIDAR Scan</span>
              </div>
              <div className="flex items-center gap-2">
                <div className="w-4 h-4 bg-yellow-500" />
                <span className="text-dark-300">Robot Heading</span>
              </div>
            </div>
          </div>

          {/* Empty columns for layout */}
          <div></div>
          <div></div>
        </div>
      </div>
    </div>
  );
}
