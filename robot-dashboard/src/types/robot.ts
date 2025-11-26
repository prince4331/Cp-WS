// Robot State Type Definitions

export type RobotMode = 'IDLE' | 'MANUAL' | 'AUTO' | 'EMERGENCY';

export type CleaningState = 
  | 'FINDING_WALL'
  | 'WALL_FOLLOWING'
  | 'TURNING_CORNER'
  | 'SPIRAL_INWARD'
  | 'EMERGENCY'
  | 'IDLE';

export interface RobotPose {
  x: number;
  y: number;
  theta: number;
}

export interface RobotVelocity {
  linear: number;
  angular: number;
}

export interface SensorData {
  ultrasonic: {
    front: number;
    left_corner: number;
    right_corner: number;
    left_side: number;
    right_side: number;
  };
  ir: {
    obstacle_front_left: boolean;
    obstacle_front_right: boolean;
    obstacle_back_left: boolean;
    obstacle_back_right: boolean;
    cliff_front_left: boolean;
    cliff_front_right: boolean;
    cliff_back_left: boolean;
    cliff_back_right: boolean;
  };
  lidar: {
    ranges: number[];
    angle_min: number;
    angle_max: number;
    angle_increment: number;
  };
  imu: {
    orientation: { x: number; y: number; z: number; w: number };
    angular_velocity: { x: number; y: number; z: number };
    linear_acceleration: { x: number; y: number; z: number };
  };
  encoders?: {
    left: number;
    right: number;
  };
  water_levels?: {
    clean: number;
    dirty: number;
  };
}

export interface BatteryInfo {
  voltage: number;
  current: number;
  percentage: number;
  charging: boolean;
  timeRemaining?: number;
}

export interface MissionStatus {
  state: CleaningState;
  progress: number;
  coverageArea: number;
  cleaningTime: number;
  distanceTraveled: number;
  spiralCount: number;
}

export interface RobotStatus {
  mode: RobotMode;
  connected: boolean;
  pose: RobotPose;
  velocity: RobotVelocity;
  sensors: SensorData;
  battery: BatteryInfo;
  mission?: MissionStatus;
  emergencyStop: boolean;
  lastUpdate: Date;
}

export interface MapData {
  width: number;
  height: number;
  resolution: number;
  origin: { x: number; y: number };
  data: number[];
}

export interface CommandVelocity {
  linear: number;
  angular: number;
}
