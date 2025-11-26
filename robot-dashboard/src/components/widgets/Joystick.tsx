import { useEffect, useRef, useState } from 'react';
import { publishVelocity } from '../../lib/rosbridge';
import { useRobotStore } from '../../store/robotStore';

export default function Joystick() {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [dragging, setDragging] = useState(false);
  const [position, setPosition] = useState({ x: 0, y: 0 });
  const maxLinearSpeed = useRobotStore((state) => state.maxLinearSpeed);
  const maxAngularSpeed = useRobotStore((state) => state.maxAngularSpeed);

  const radius = 80;
  const knobRadius = 30;

  // Continuous velocity publishing
  useEffect(() => {
    const maxDistance = radius - knobRadius;
    const linear = (-position.y / maxDistance) * maxLinearSpeed;
    const angular = (-position.x / maxDistance) * maxAngularSpeed;

    // Publish velocity continuously at 10Hz while joystick is active
    const interval = setInterval(() => {
      if (dragging || (position.x !== 0 || position.y !== 0)) {
        publishVelocity(linear, angular);
      }
    }, 100); // 10Hz update rate

    return () => clearInterval(interval);
  }, [position, dragging, maxLinearSpeed, maxAngularSpeed, radius, knobRadius]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Clear canvas
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    const centerX = canvas.width / 2;
    const centerY = canvas.height / 2;

    // Draw outer circle
    ctx.strokeStyle = '#334155';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(centerX, centerY, radius, 0, 2 * Math.PI);
    ctx.stroke();

    // Draw center crosshair
    ctx.strokeStyle = '#475569';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(centerX - 10, centerY);
    ctx.lineTo(centerX + 10, centerY);
    ctx.moveTo(centerX, centerY - 10);
    ctx.lineTo(centerX, centerY + 10);
    ctx.stroke();

    // Draw knob
    const knobX = centerX + position.x;
    const knobY = centerY + position.y;

    ctx.fillStyle = dragging ? '#0ea5e9' : '#1e40af';
    ctx.beginPath();
    ctx.arc(knobX, knobY, knobRadius, 0, 2 * Math.PI);
    ctx.fill();

    ctx.strokeStyle = '#60a5fa';
    ctx.lineWidth = 2;
    ctx.stroke();
  }, [position, dragging]);

  const handlePointerDown = (e: React.PointerEvent<HTMLCanvasElement>) => {
    setDragging(true);
    handlePointerMove(e);
  };

  const handlePointerMove = (e: React.PointerEvent<HTMLCanvasElement>) => {
    if (!dragging && e.buttons !== 1) return;

    const canvas = canvasRef.current;
    if (!canvas) return;

    const rect = canvas.getBoundingClientRect();
    const centerX = canvas.width / 2;
    const centerY = canvas.height / 2;

    const x = e.clientX - rect.left - centerX;
    const y = e.clientY - rect.top - centerY;

    // Limit to radius
    const distance = Math.sqrt(x * x + y * y);
    const maxDistance = radius - knobRadius;

    if (distance > maxDistance) {
      const angle = Math.atan2(y, x);
      setPosition({
        x: Math.cos(angle) * maxDistance,
        y: Math.sin(angle) * maxDistance,
      });
    } else {
      setPosition({ x, y });
    }

    // Velocity publishing is handled by the continuous interval
  };

  const handlePointerUp = () => {
    setDragging(false);
    setPosition({ x: 0, y: 0 });
    publishVelocity(0, 0);
  };

  return (
    <div className="flex flex-col items-center gap-4">
      <canvas
        ref={canvasRef}
        width={200}
        height={200}
        onPointerDown={handlePointerDown}
        onPointerMove={handlePointerMove}
        onPointerUp={handlePointerUp}
        onPointerLeave={handlePointerUp}
        className="cursor-pointer touch-none bg-dark-900 rounded-lg border border-dark-800"
      />
      <div className="text-sm text-dark-400 text-center">
        <p>Drag to control robot</p>
        <p className="text-xs mt-1">Forward/Back • Left/Right</p>
      </div>
    </div>
  );
}
