// ROS Message Type Definitions

export interface LaserScan {
  header: {
    stamp: { sec: number; nanosec: number };
    frame_id: string;
  };
  angle_min: number;
  angle_max: number;
  angle_increment: number;
  time_increment: number;
  scan_time: number;
  range_min: number;
  range_max: number;
  ranges: number[];
  intensities: number[];
}

export interface Odometry {
  header: {
    stamp: { sec: number; nanosec: number };
    frame_id: string;
  };
  child_frame_id: string;
  pose: {
    pose: {
      position: { x: number; y: number; z: number };
      orientation: { x: number; y: number; z: number; w: number };
    };
    covariance: number[];
  };
  twist: {
    twist: {
      linear: { x: number; y: number; z: number };
      angular: { x: number; y: number; z: number };
    };
    covariance: number[];
  };
}

export interface Twist {
  linear: { x: number; y: number; z: number };
  angular: { x: number; y: number; z: number };
}

export interface OccupancyGrid {
  header: {
    stamp: { sec: number; nanosec: number };
    frame_id: string;
  };
  info: {
    map_load_time: { sec: number; nanosec: number };
    resolution: number;
    width: number;
    height: number;
    origin: {
      position: { x: number; y: number; z: number };
      orientation: { x: number; y: number; z: number; w: number };
    };
  };
  data: number[];
}

export interface BatteryState {
  header: {
    stamp: { sec: number; nanosec: number };
    frame_id: string;
  };
  voltage: number;
  current: number;
  charge: number;
  capacity: number;
  design_capacity: number;
  percentage: number;
  power_supply_status: number;
  power_supply_health: number;
  power_supply_technology: number;
  present: boolean;
}

export interface UltrasonicSensors {
  header: {
    stamp: { sec: number; nanosec: number };
    frame_id: string;
  };
  front: number;
  left_corner: number;
  right_corner: number;
  left_side: number;
  right_side: number;
}

export interface IRSensors {
  header: {
    stamp: { sec: number; nanosec: number };
    frame_id: string;
  };
  obstacle_front_left: boolean;
  obstacle_front_right: boolean;
  obstacle_back_left: boolean;
  obstacle_back_right: boolean;
  cliff_front_left: boolean;
  cliff_front_right: boolean;
  cliff_back_left: boolean;
  cliff_back_right: boolean;
}

export interface IMU {
  header: {
    stamp: { sec: number; nanosec: number };
    frame_id: string;
  };
  orientation: { x: number; y: number; z: number; w: number };
  orientation_covariance: number[];
  angular_velocity: { x: number; y: number; z: number };
  angular_velocity_covariance: number[];
  linear_acceleration: { x: number; y: number; z: number };
  linear_acceleration_covariance: number[];
}

export interface CleaningStatus {
  state: string;
  progress: number;
  coverage_area: number;
  cleaning_time: number;
  distance_traveled: number;
}
