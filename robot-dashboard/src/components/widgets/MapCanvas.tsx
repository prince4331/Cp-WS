import { useEffect, useRef, useState } from 'react';
import { useRobotStore } from '../../store/robotStore';

interface MapCanvasProps {
  width?: number;
  height?: number;
  showLidar?: boolean;
  showRobot?: boolean;
}

export default function MapCanvas({
  width = 600,
  height = 600,
  showLidar = true,
  showRobot = true,
}: MapCanvasProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const parentRef = useRef<HTMLDivElement>(null);
  const mapData = useRobotStore((state) => state.mapData);
  const robotPose = useRobotStore((state) => state.robotStatus.pose);
  const headingYaw = useRobotStore((state) => state.heading.yaw || state.robotStatus.pose.theta);
  const lidarRanges = useRobotStore((state) => state.lidarRanges);
  const lidarAngleMin = useRobotStore((state) => state.lidarAngleMin);
  const lidarAngleIncrement = useRobotStore((state) => state.lidarAngleIncrement);
  const [canvasSize, setCanvasSize] = useState({ width, height });

  useEffect(() => {
    if (!parentRef.current) return;

    const updateSize = () => {
      const parentWidth = parentRef.current?.clientWidth || width;
      const computedWidth = Math.min(parentWidth, width);
      const ratio = height / width;
      setCanvasSize({
        width: computedWidth,
        height: computedWidth * ratio,
      });
    };

    updateSize();
    const observer = new ResizeObserver(updateSize);
    observer.observe(parentRef.current);
    return () => observer.disconnect();
  }, [width, height]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const dpr = window.devicePixelRatio || 1;
    canvas.width = canvasSize.width * dpr;
    canvas.height = canvasSize.height * dpr;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

    // Clear canvas
    ctx.fillStyle = '#020617';
    ctx.fillRect(0, 0, canvasSize.width, canvasSize.height);

    // Draw map if available
    if (mapData) {
      const scale = Math.min(
        canvasSize.width / mapData.width,
        canvasSize.height / mapData.height
      );
      const offsetX = (canvasSize.width - mapData.width * scale) / 2;
      const offsetY = (canvasSize.height - mapData.height * scale) / 2;

      // Draw occupancy grid
      for (let y = 0; y < mapData.height; y++) {
        for (let x = 0; x < mapData.width; x++) {
          const index = y * mapData.width + x;
          const value = mapData.data[index];

          if (value === -1) {
            ctx.fillStyle = '#1e293b'; // Unknown
          } else if (value > 50) {
            ctx.fillStyle = '#475569'; // Obstacle
          } else {
            ctx.fillStyle = '#0f172a'; // Free
          }

          ctx.fillRect(
            offsetX + x * scale,
            offsetY + (mapData.height - y - 1) * scale,
            scale,
            scale
          );
        }
      }

      // Convert robot pose to map coordinates
      const robotMapX = (robotPose.x - mapData.origin.x) / mapData.resolution;
      const robotMapY = (robotPose.y - mapData.origin.y) / mapData.resolution;
      const robotScreenX = offsetX + robotMapX * scale;
      const robotScreenY = offsetY + (mapData.height - robotMapY) * scale;

      // Draw LIDAR scan
      if (showLidar && lidarRanges.length > 0) {
        ctx.strokeStyle = '#0ea5e9';
        ctx.lineWidth = 1;
        ctx.beginPath();

        for (let i = 0; i < lidarRanges.length; i += 5) {
          const range = lidarRanges[i];
          if (range < 0.1 || range > 10) continue;

          const angle = lidarAngleMin + i * lidarAngleIncrement + headingYaw;
          const endX = robotScreenX + (range / mapData.resolution) * Math.cos(angle) * scale;
          const endY = robotScreenY - (range / mapData.resolution) * Math.sin(angle) * scale;

          ctx.moveTo(robotScreenX, robotScreenY);
          ctx.lineTo(endX, endY);
        }

        ctx.stroke();
      }

      // Draw robot
      if (showRobot) {
        // Robot body (circle)
        ctx.fillStyle = '#0ea5e9';
        ctx.beginPath();
        ctx.arc(robotScreenX, robotScreenY, 10, 0, 2 * Math.PI);
        ctx.fill();

        // Robot heading arrow
        const arrowLength = 20;
        const arrowEndX = robotScreenX + arrowLength * Math.cos(headingYaw);
        const arrowEndY = robotScreenY - arrowLength * Math.sin(headingYaw);

        ctx.strokeStyle = '#fbbf24';
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(robotScreenX, robotScreenY);
        ctx.lineTo(arrowEndX, arrowEndY);
        ctx.stroke();

        // Arrow head
        const headSize = 6;
        ctx.beginPath();
        ctx.moveTo(arrowEndX, arrowEndY);
        ctx.lineTo(
          arrowEndX - headSize * Math.cos(headingYaw - Math.PI / 6),
          arrowEndY + headSize * Math.sin(headingYaw - Math.PI / 6)
        );
        ctx.moveTo(arrowEndX, arrowEndY);
        ctx.lineTo(
          arrowEndX - headSize * Math.cos(headingYaw + Math.PI / 6),
          arrowEndY + headSize * Math.sin(headingYaw + Math.PI / 6)
        );
        ctx.stroke();
      }
    } else {
      // No map data - show message
      ctx.fillStyle = '#64748b';
      ctx.font = '16px sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText('Waiting for map data...', canvasSize.width / 2, canvasSize.height / 2);
    }
  }, [
    mapData,
    robotPose,
    lidarRanges,
    headingYaw,
    lidarAngleMin,
    lidarAngleIncrement,
    canvasSize.width,
    canvasSize.height,
    showLidar,
    showRobot,
    width,
    height,
  ]);

  return (
    <div ref={parentRef} className="w-full">
      <canvas
        ref={canvasRef}
        style={{ width: canvasSize.width, height: canvasSize.height }}
        className="rounded-lg border border-dark-800 w-full h-auto"
      />
    </div>
  );
}
