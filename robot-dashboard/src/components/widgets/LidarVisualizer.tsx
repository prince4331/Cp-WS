import { useEffect, useRef, useState } from 'react';
import { useRobotStore } from '../../store/robotStore';

interface LidarVisualizerProps {
  size?: number;
}

export default function LidarVisualizer({ size = 300 }: LidarVisualizerProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const parentRef = useRef<HTMLDivElement>(null);
  const lidarRanges = useRobotStore((state) => state.lidarRanges);
  const lidarAngleMin = useRobotStore((state) => state.lidarAngleMin);
  const lidarAngleIncrement = useRobotStore((state) => state.lidarAngleIncrement);
  const headingYaw = useRobotStore((state) => state.heading.yaw || state.robotStatus.pose.theta);
  const [canvasSize, setCanvasSize] = useState(size);

  useEffect(() => {
    if (!parentRef.current) return;

    const updateSize = () => {
      const parentWidth = parentRef.current?.clientWidth || size;
      setCanvasSize(Math.min(parentWidth, size));
    };

    updateSize();
    const observer = new ResizeObserver(updateSize);
    observer.observe(parentRef.current);
    return () => observer.disconnect();
  }, [size]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const dpr = window.devicePixelRatio || 1;
    canvas.width = canvasSize * dpr;
    canvas.height = canvasSize * dpr;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

    // Clear canvas
    ctx.fillStyle = '#020617';
    ctx.fillRect(0, 0, canvasSize, canvasSize);

    const centerX = canvasSize / 2;
    const centerY = canvasSize / 2;
    const scale = canvasSize / 8; // 4 meters visible radius

    // Draw grid circles
    ctx.strokeStyle = '#1e293b';
    ctx.lineWidth = 1;
    for (let r = 1; r <= 4; r++) {
      ctx.beginPath();
      ctx.arc(centerX, centerY, r * scale, 0, 2 * Math.PI);
      ctx.stroke();
    }

    // Draw center axes
    ctx.strokeStyle = '#334155';
    ctx.beginPath();
    ctx.moveTo(centerX, 0);
    ctx.lineTo(centerX, size);
    ctx.moveTo(0, centerY);
    ctx.lineTo(size, centerY);
    ctx.stroke();

    // Draw LIDAR points
    if (lidarRanges.length > 0) {
      ctx.fillStyle = '#0ea5e9';
      
      for (let i = 0; i < lidarRanges.length; i += 2) {
        const range = lidarRanges[i];
        if (range < 0.1 || range > 4) continue;

        const angle = lidarAngleMin + i * lidarAngleIncrement + headingYaw;
        const x = centerX + range * scale * Math.cos(angle);
        const y = centerY - range * scale * Math.sin(angle);

        ctx.beginPath();
        ctx.arc(x, y, 2, 0, 2 * Math.PI);
        ctx.fill();
      }
    }

    // Draw robot center
    ctx.fillStyle = '#fbbf24';
    ctx.beginPath();
    ctx.arc(centerX, centerY, 5, 0, 2 * Math.PI);
    ctx.fill();

    // Draw forward indicator arrow
    ctx.strokeStyle = '#fbbf24';
    ctx.lineWidth = 3;
    const arrowLength = canvasSize * 0.2;
    const arrowX = centerX + Math.cos(headingYaw) * arrowLength;
    const arrowY = centerY - Math.sin(headingYaw) * arrowLength;
    ctx.beginPath();
    ctx.moveTo(centerX, centerY);
    ctx.lineTo(arrowX, arrowY);
    ctx.stroke();

    // Arrow head
    const headSize = 10;
    ctx.beginPath();
    ctx.moveTo(arrowX, arrowY);
    ctx.lineTo(
      arrowX - headSize * Math.cos(headingYaw - Math.PI / 6),
      arrowY + headSize * Math.sin(headingYaw - Math.PI / 6)
    );
    ctx.moveTo(arrowX, arrowY);
    ctx.lineTo(
      arrowX - headSize * Math.cos(headingYaw + Math.PI / 6),
      arrowY + headSize * Math.sin(headingYaw + Math.PI / 6)
    );
    ctx.stroke();

  }, [lidarRanges, lidarAngleMin, lidarAngleIncrement, canvasSize, headingYaw]);

  return (
    <div ref={parentRef} className="w-full">
      <canvas
        ref={canvasRef}
        style={{ width: canvasSize, height: canvasSize }}
        className="rounded-lg border border-dark-800 w-full h-auto"
      />
    </div>
  );
}
