import { create } from 'zustand';
import type { RobotStatus, MapData, SensorData, RobotMode, CleaningState } from '../types/robot';

type HeadingSource = 'imu' | 'odom';

interface HeadingState {
  yaw: number;
  source: HeadingSource;
  timestamp: number;
}

const normalizeAngle = (angle: number) => {
  let a = angle;
  while (a > Math.PI) {
    a -= 2 * Math.PI;
  }
  while (a < -Math.PI) {
    a += 2 * Math.PI;
  }
  return a;
};

const MAG_DECLINATION_RAD = (-0.3833333333 * Math.PI) / 180; // -0°23' ≈ -0.383°

interface RobotStore {
  // Connection state
  connected: boolean;
  setConnected: (connected: boolean) => void;

  // Robot status
  robotStatus: RobotStatus;
  updatePose: (x: number, y: number, theta: number) => void;
  updateVelocity: (linear: number, angular: number) => void;
  updateBattery: (voltage: number, current: number, percentage: number, charging: boolean) => void;
  updateSensors: (sensors: Partial<SensorData>) => void;
  updateMode: (mode: RobotMode) => void;
  updateMissionStatus: (state: CleaningState, progress: number, coverage: number, time: number, distance: number) => void;
  setEmergencyStop: (stop: boolean) => void;
  setMissionState: (state: CleaningState) => void;

  heading: HeadingState;
  setHeading: (yaw: number, source: HeadingSource) => void;

  // Cleaning systems state
  cleaningSystems: {
    vacuumPump: boolean;
    scrubber: boolean;
    sweepingBrush: boolean;
    waterPump: boolean;
  };
  updateCleaningSystem: (system: 'vacuumPump' | 'scrubber' | 'sweepingBrush' | 'waterPump', state: boolean) => void;

  // Map data
  mapData: MapData | null;
  updateMap: (map: MapData) => void;

  // LIDAR data for visualization
  lidarRanges: number[];
  lidarAngleMin: number;
  lidarAngleMax: number;
  lidarAngleIncrement: number;
  updateLidar: (ranges: number[], angleMin: number, angleMax: number, angleIncrement: number) => void;

  // Settings
  maxLinearSpeed: number;
  maxAngularSpeed: number;
  setMaxLinearSpeed: (speed: number) => void;
  setMaxAngularSpeed: (speed: number) => void;
}

export const useRobotStore = create<RobotStore>((set) => ({
  // Initial connection state
  connected: false,
  setConnected: (connected) => set({ connected }),

  // Initial robot status
  robotStatus: {
    mode: 'IDLE',
    connected: false,
    pose: { x: 0, y: 0, theta: 0 },
    velocity: { linear: 0, angular: 0 },
    sensors: {
      ultrasonic: {
        front: 0,
        left_corner: 0,
        right_corner: 0,
        left_side: 0,
        right_side: 0,
      },
      ir: {
        obstacle_front_left: false,
        obstacle_front_right: false,
        obstacle_back_left: false,
        obstacle_back_right: false,
        cliff_front_left: false,
        cliff_front_right: false,
        cliff_back_left: false,
        cliff_back_right: false,
      },
      lidar: {
        ranges: [],
        angle_min: 0,
        angle_max: 0,
        angle_increment: 0,
      },
      imu: {
        orientation: { x: 0, y: 0, z: 0, w: 1 },
        angular_velocity: { x: 0, y: 0, z: 0 },
        linear_acceleration: { x: 0, y: 0, z: 0 },
      },
      encoders: {
        left: 0,
        right: 0,
      },
      water_levels: {
        clean: 0,
        dirty: 0,
      },
    },
    battery: {
      voltage: 12.0,
      current: 0,
      percentage: 100,
      charging: false,
    },
    mission: {
      state: 'IDLE',
      progress: 0,
      coverageArea: 0,
      cleaningTime: 0,
      distanceTraveled: 0,
      spiralCount: 0,
    },
    emergencyStop: false,
    lastUpdate: new Date(),
  },

  heading: {
    yaw: 0,
    source: 'odom',
    timestamp: 0,
  },

  updatePose: (x, y, theta) =>
    set((state) => {
      const now = Date.now();
      const headingStale = now - state.heading.timestamp > 2000;
      const fallbackYaw = normalizeAngle(theta + MAG_DECLINATION_RAD);
      return {
        robotStatus: {
          ...state.robotStatus,
          pose: { x, y, theta },
          lastUpdate: new Date(),
        },
        heading: headingStale
          ? {
              yaw: fallbackYaw,
              source: 'odom',
              timestamp: now,
            }
          : state.heading,
      };
    }),

  updateVelocity: (linear, angular) =>
    set((state) => ({
      robotStatus: {
        ...state.robotStatus,
        velocity: { linear, angular },
        lastUpdate: new Date(),
      },
    })),

  updateBattery: (voltage, current, percentage, charging) =>
    set((state) => ({
      robotStatus: {
        ...state.robotStatus,
        battery: { voltage, current, percentage, charging },
        lastUpdate: new Date(),
      },
    })),

  updateSensors: (sensors) =>
    set((state) => ({
      robotStatus: {
        ...state.robotStatus,
        sensors: {
          ...state.robotStatus.sensors,
          ...sensors,
        },
        lastUpdate: new Date(),
      },
    })),

  updateMode: (mode) =>
    set((state) => ({
      robotStatus: {
        ...state.robotStatus,
        mode,
        lastUpdate: new Date(),
      },
    })),

  updateMissionStatus: (state_value, progress, coverage, time, distance) =>
    set((state) => ({
      robotStatus: {
        ...state.robotStatus,
        mission: {
          state: state_value,
          progress,
          coverageArea: coverage,
          cleaningTime: time,
          distanceTraveled: distance,
          spiralCount: state.robotStatus.mission?.spiralCount || 0,
        },
        lastUpdate: new Date(),
      },
    })),

  setEmergencyStop: (stop) =>
    set((state) => ({
      robotStatus: {
        ...state.robotStatus,
        emergencyStop: stop,
        lastUpdate: new Date(),
      },
    })),

  setMissionState: (state_value) =>
    set((state) => ({
      robotStatus: {
        ...state.robotStatus,
        mission: state.robotStatus.mission
          ? {
              ...state.robotStatus.mission,
              state: state_value,
            }
          : {
              state: state_value,
              progress: 0,
              coverageArea: 0,
              cleaningTime: 0,
              distanceTraveled: 0,
              spiralCount: 0,
            },
        lastUpdate: new Date(),
      },
    })),

  setHeading: (yaw, source) =>
    set((state) => {
      const now = Date.now();
      const magneticYaw = normalizeAngle(yaw + MAG_DECLINATION_RAD);
      return {
        heading: {
          yaw: magneticYaw,
          source,
          timestamp: now,
        },
        robotStatus: {
          ...state.robotStatus,
          lastUpdate: new Date(),
        },
      };
    }),

  // Cleaning systems
  cleaningSystems: {
    vacuumPump: false,
    scrubber: false,
    sweepingBrush: false,
    waterPump: false,
  },
  updateCleaningSystem: (system, state_value) =>
    set((state) => ({
      cleaningSystems: {
        ...state.cleaningSystems,
        [system]: state_value,
      },
    })),

  // Map data
  mapData: null,
  updateMap: (map) => set({ mapData: map }),

  // LIDAR data
  lidarRanges: [],
  lidarAngleMin: 0,
  lidarAngleMax: 0,
  lidarAngleIncrement: 0,
  updateLidar: (ranges, angleMin, angleMax, angleIncrement) =>
    set({
      lidarRanges: ranges,
      lidarAngleMin: angleMin,
      lidarAngleMax: angleMax,
      lidarAngleIncrement: angleIncrement,
    }),

  // Settings
  maxLinearSpeed: 0.3,
  maxAngularSpeed: 1.0,
  setMaxLinearSpeed: (speed) => set({ maxLinearSpeed: speed }),
  setMaxAngularSpeed: (speed) => set({ maxAngularSpeed: speed }),
}));
