import ROSLIB from 'roslib';
import type { 
  LaserScan, 
  Odometry, 
  OccupancyGrid, 
  BatteryState,
  UltrasonicSensors,
  IRSensors,
  IMU,
  Twist
} from '../types/ros';

const DEFAULT_ROSBRIDGE_URL =
  (import.meta.env.VITE_ROSBRIDGE_URL && import.meta.env.VITE_ROSBRIDGE_URL.trim() !== '')
    ? import.meta.env.VITE_ROSBRIDGE_URL
    : 'ws://192.168.163.134:9090';

// ROS Bridge Connection Manager
class ROSBridge {
  private ros: ROSLIB.Ros | null = null;
  private reconnectInterval: NodeJS.Timeout | null = null;
  private connectionCallbacks: ((connected: boolean) => void)[] = [];
  
  constructor(private url: string = DEFAULT_ROSBRIDGE_URL) {}

  connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      this.ros = new ROSLIB.Ros({ url: this.url });

      this.ros.on('connection', () => {
        console.log('✅ Connected to ROS bridge');
        this.notifyConnectionChange(true);
        if (this.reconnectInterval) {
          clearInterval(this.reconnectInterval);
          this.reconnectInterval = null;
        }
        resolve();
      });

      this.ros.on('error', (error) => {
        console.error('❌ ROS connection error:', error);
        this.notifyConnectionChange(false);
        reject(error);
      });

      this.ros.on('close', () => {
        console.warn('⚠️ ROS connection closed');
        this.notifyConnectionChange(false);
        this.attemptReconnect();
      });
    });
  }

  private attemptReconnect() {
    if (this.reconnectInterval) return;
    
    this.reconnectInterval = setInterval(() => {
      console.log('🔄 Attempting to reconnect...');
      this.connect().catch(() => {});
    }, 5000);
  }

  disconnect() {
    if (this.reconnectInterval) {
      clearInterval(this.reconnectInterval);
      this.reconnectInterval = null;
    }
    if (this.ros) {
      this.ros.close();
      this.ros = null;
    }
  }

  isConnected(): boolean {
    return this.ros !== null && this.ros.isConnected;
  }

  onConnectionChange(callback: (connected: boolean) => void) {
    this.connectionCallbacks.push(callback);
    return () => {
      this.connectionCallbacks = this.connectionCallbacks.filter(cb => cb !== callback);
    };
  }

  private notifyConnectionChange(connected: boolean) {
    this.connectionCallbacks.forEach(cb => cb(connected));
  }

  // Subscribe to a topic
  subscribeTo<T>(
    topicName: string,
    messageType: string,
    callback: (message: T) => void,
    throttleRate?: number
  ): ROSLIB.Topic | null {
    if (!this.ros) return null;

    const topic = new ROSLIB.Topic({
      ros: this.ros,
      name: topicName,
      messageType: messageType,
      throttle_rate: throttleRate,
    });

    topic.subscribe((message) => {
      callback(message as T);
    });

    return topic;
  }

  // Publish to a topic
  publishTo<T>(
    topicName: string,
    messageType: string,
    message: T
  ): void {
    if (!this.ros) return;

    const topic = new ROSLIB.Topic({
      ros: this.ros,
      name: topicName,
      messageType: messageType,
    });

    const rosMessage = new ROSLIB.Message(message as any);
    topic.publish(rosMessage);
  }

  // Call a service
  callService<TRequest, TResponse>(
    serviceName: string,
    serviceType: string,
    request: TRequest
  ): Promise<TResponse> {
    return new Promise((resolve, reject) => {
      if (!this.ros) {
        reject(new Error('Not connected to ROS'));
        return;
      }

      const service = new ROSLIB.Service({
        ros: this.ros,
        name: serviceName,
        serviceType: serviceType,
      });

      const serviceRequest = new ROSLIB.ServiceRequest(request as any);
      
      service.callService(
        serviceRequest,
        (response) => resolve(response as TResponse),
        (error) => reject(error)
      );
    });
  }
}

// Singleton instance
export const rosbridge = new ROSBridge();

// Topic subscription helpers
export function subscribeLaserScan(callback: (scan: LaserScan) => void) {
  return rosbridge.subscribeTo<LaserScan>('/scan', 'sensor_msgs/LaserScan', callback, 100);
}

export function subscribeOdometry(callback: (odom: Odometry) => void) {
  return rosbridge.subscribeTo<Odometry>('/odom', 'nav_msgs/Odometry', callback, 100);
}

export function subscribeMap(callback: (map: OccupancyGrid) => void) {
  console.log('🗺️  Setting up /map subscription...');
  return rosbridge.subscribeTo<OccupancyGrid>('/map', 'nav_msgs/OccupancyGrid', (map) => {
    console.log('📍 Map data received:', {
      width: map.info?.width,
      height: map.info?.height,
      dataLength: map.data?.length,
      resolution: map.info?.resolution
    });
    callback(map);
  }, 1000);
}

export function subscribeBattery(callback: (battery: BatteryState) => void) {
  return rosbridge.subscribeTo<BatteryState>('/battery/state', 'sensor_msgs/BatteryState', callback, 1000);
}

export function subscribeUltrasonic(callback: (sensors: UltrasonicSensors) => void) {
  // Subscribe to individual ultrasonic Range topics and aggregate them
  const aggregatedData: any = {
    front: 0,
    left_corner: 0,
    right_corner: 0,
    left_side: 0,
    right_side: 0
  };

  const frontTopic = rosbridge.subscribeTo<any>('/ultrasonic/front', 'sensor_msgs/Range', (msg) => {
    aggregatedData.front = msg.range * 100; // Convert meters to cm
    callback(aggregatedData);
  }, 100);

  rosbridge.subscribeTo<any>('/ultrasonic/front_left', 'sensor_msgs/Range', (msg) => {
    aggregatedData.left_corner = msg.range * 100;
    callback(aggregatedData);
  }, 100);

  rosbridge.subscribeTo<any>('/ultrasonic/front_right', 'sensor_msgs/Range', (msg) => {
    aggregatedData.right_corner = msg.range * 100;
    callback(aggregatedData);
  }, 100);

  rosbridge.subscribeTo<any>('/ultrasonic/left', 'sensor_msgs/Range', (msg) => {
    aggregatedData.left_side = msg.range * 100;
    callback(aggregatedData);
  }, 100);

  rosbridge.subscribeTo<any>('/ultrasonic/right', 'sensor_msgs/Range', (msg) => {
    aggregatedData.right_side = msg.range * 100;
    callback(aggregatedData);
  }, 100);

  return frontTopic; // Return one topic for unsubscribe (others will be cleaned up too)
}

export function subscribeIR(callback: (sensors: IRSensors) => void) {
  const aggregatedData: IRSensors = {
    header: {
      stamp: { sec: 0, nanosec: 0 },
      frame_id: 'ir/aggregate',
    },
    obstacle_front_left: false,
    obstacle_front_right: false,
    obstacle_back_left: false,
    obstacle_back_right: false,
    cliff_front_left: false,
    cliff_front_right: false,
    cliff_back_left: false,
    cliff_back_right: false,
  };

  const emitUpdate = () => callback({ ...aggregatedData });

  const frontLeftObjTopic = rosbridge.subscribeTo<{ data: boolean }>(
    '/ir/front_left/object',
    'std_msgs/Bool',
    (msg) => {
      aggregatedData.obstacle_front_left = msg.data;
      emitUpdate();
    },
    100
  );

  rosbridge.subscribeTo<{ data: boolean }>(
    '/ir/front_right/object',
    'std_msgs/Bool',
    (msg) => {
      aggregatedData.obstacle_front_right = msg.data;
      emitUpdate();
    },
    100
  );

  rosbridge.subscribeTo<{ data: boolean }>(
    '/ir/back_left/object',
    'std_msgs/Bool',
    (msg) => {
      aggregatedData.obstacle_back_left = msg.data;
      emitUpdate();
    },
    100
  );

  rosbridge.subscribeTo<{ data: boolean }>(
    '/ir/back_right/object',
    'std_msgs/Bool',
    (msg) => {
      aggregatedData.obstacle_back_right = msg.data;
      emitUpdate();
    },
    100
  );

  rosbridge.subscribeTo<{ data: boolean }>(
    '/ir/front_left/stair',
    'std_msgs/Bool',
    (msg) => {
      aggregatedData.cliff_front_left = msg.data;
      emitUpdate();
    },
    100
  );

  rosbridge.subscribeTo<{ data: boolean }>(
    '/ir/front_right/stair',
    'std_msgs/Bool',
    (msg) => {
      aggregatedData.cliff_front_right = msg.data;
      emitUpdate();
    },
    100
  );

  rosbridge.subscribeTo<{ data: boolean }>(
    '/ir/back_left/stair',
    'std_msgs/Bool',
    (msg) => {
      aggregatedData.cliff_back_left = msg.data;
      emitUpdate();
    },
    100
  );

  rosbridge.subscribeTo<{ data: boolean }>(
    '/ir/back_right/stair',
    'std_msgs/Bool',
    (msg) => {
      aggregatedData.cliff_back_right = msg.data;
      emitUpdate();
    },
    100
  );

  return frontLeftObjTopic;
}

export function subscribeIMU(callback: (imu: IMU) => void) {
  return rosbridge.subscribeTo<IMU>('/imu/data', 'sensor_msgs/Imu', callback, 100);
}

export function subscribeRelayStatus(callback: (relay: number, state: boolean) => void) {
  return rosbridge.subscribeTo<{ data: string }>(
    '/relay/status',
    'std_msgs/String',
    (msg) => {
      if (!msg?.data) return;
      const parts = msg.data.split(',');
      if (parts.length < 2) return;
      const relay = parseInt(parts[0], 10);
      const state = parseInt(parts[1], 10);
      if (!Number.isFinite(relay) || !Number.isFinite(state)) return;
      callback(relay, state === 1);
    },
    200
  );
}

export function subscribeEncoders(callback: (left: number, right: number) => void) {
  const encoders = { left: 0, right: 0 };
  const emitUpdate = () => callback(encoders.left, encoders.right);

  const leftTopic = rosbridge.subscribeTo<{ data: number }>(
    '/encoder/left',
    'std_msgs/Int32',
    (msg) => {
      encoders.left = msg.data ?? 0;
      emitUpdate();
    },
    100
  );

  rosbridge.subscribeTo<{ data: number }>(
    '/encoder/right',
    'std_msgs/Int32',
    (msg) => {
      encoders.right = msg.data ?? 0;
      emitUpdate();
    },
    100
  );

  return leftTopic;
}

export function subscribeWaterLevels(callback: (clean: number, dirty: number) => void) {
  const levels = { clean: 0, dirty: 0 };
  const emitUpdate = () => callback(levels.clean, levels.dirty);

  const cleanTopic = rosbridge.subscribeTo<{ range: number }>(
    '/water_level/clean',
    'sensor_msgs/Range',
    (msg) => {
      if (typeof msg?.range === 'number') {
        levels.clean = msg.range * 100; // convert meters to cm
        emitUpdate();
      }
    },
    500
  );

  rosbridge.subscribeTo<{ range: number }>(
    '/water_level/dirty',
    'sensor_msgs/Range',
    (msg) => {
      if (typeof msg?.range === 'number') {
        levels.dirty = msg.range * 100;
        emitUpdate();
      }
    },
    500
  );

  return cleanTopic;
}

export function publishVelocity(linear: number, angular: number) {
  const twist: Twist = {
    linear: { x: linear, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: angular },
  };
  rosbridge.publishTo<Twist>('/cmd_vel', 'geometry_msgs/Twist', twist);
}

export function publishRelayCommand(relay: number, state: boolean) {
  const payload = `${relay},${state ? 1 : 0}`;
  rosbridge.publishTo<{ data: string }>(
    '/relay/command',
    'std_msgs/String',
    { data: payload }
  );
}

export function publishEmergencyStop(active: boolean) {
  rosbridge.publishTo<{ data: boolean }>(
    '/emergency_stop',
    'std_msgs/Bool',
    { data: active }
  );
}

type CleaningCommand = 'start' | 'pause' | 'stop';

export function publishCleaningCommand(command: CleaningCommand) {
  rosbridge.publishTo<{ data: string }>(
    '/cleaning_command',
    'std_msgs/String',
    { data: command }
  );
}

export default rosbridge;
