import ROSLIB from 'roslib';
import type { 
  LaserScan, 
  Odometry, 
  OccupancyGrid, 
  BatteryState,
  UltrasonicSensors,
  IRSensors,
  IMU,
  CleaningStatus,
  Twist
} from '../types/ros';

// ROS Bridge Connection Manager
class ROSBridge {
  private ros: ROSLIB.Ros | null = null;
  private reconnectInterval: NodeJS.Timeout | null = null;
  private connectionCallbacks: ((connected: boolean) => void)[] = [];
  
  constructor(private url: string = 'ws://localhost:9090') {}

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
  // Subscribe to individual IR Bool topics and aggregate them
  const aggregatedData: any = {
    front_left_object: false,
    front_left_stair: false,
    front_right_object: false,
    front_right_stair: false,
    back_left_object: false,
    back_left_stair: false,
    back_right_object: false,
    back_right_stair: false
  };

  const frontLeftObjTopic = rosbridge.subscribeTo<any>('/ir/obstacle/front_left/object', 'std_msgs/Bool', (msg) => {
    aggregatedData.front_left_object = msg.data;
    callback(aggregatedData);
  }, 100);

  rosbridge.subscribeTo<any>('/ir/stair/front_left', 'std_msgs/Bool', (msg) => {
    aggregatedData.front_left_stair = msg.data;
    callback(aggregatedData);
  }, 100);

  rosbridge.subscribeTo<any>('/ir/obstacle/front_right/object', 'std_msgs/Bool', (msg) => {
    aggregatedData.front_right_object = msg.data;
    callback(aggregatedData);
  }, 100);

  rosbridge.subscribeTo<any>('/ir/stair/front_right', 'std_msgs/Bool', (msg) => {
    aggregatedData.front_right_stair = msg.data;
    callback(aggregatedData);
  }, 100);

  rosbridge.subscribeTo<any>('/ir/obstacle/back_left/object', 'std_msgs/Bool', (msg) => {
    aggregatedData.back_left_object = msg.data;
    callback(aggregatedData);
  }, 100);

  rosbridge.subscribeTo<any>('/ir/stair/back_left', 'std_msgs/Bool', (msg) => {
    aggregatedData.back_left_stair = msg.data;
    callback(aggregatedData);
  }, 100);

  rosbridge.subscribeTo<any>('/ir/obstacle/back_right/object', 'std_msgs/Bool', (msg) => {
    aggregatedData.back_right_object = msg.data;
    callback(aggregatedData);
  }, 100);

  rosbridge.subscribeTo<any>('/ir/stair/back_right', 'std_msgs/Bool', (msg) => {
    aggregatedData.back_right_stair = msg.data;
    callback(aggregatedData);
  }, 100);

  return frontLeftObjTopic; // Return one topic for unsubscribe
}

export function subscribeIMU(callback: (imu: IMU) => void) {
  return rosbridge.subscribeTo<IMU>('/imu/data', 'sensor_msgs/Imu', callback, 100);
}

export function subscribeCleaningStatus(callback: (status: CleaningStatus) => void) {
  return rosbridge.subscribeTo<CleaningStatus>(
    '/cleaning_status',
    'std_msgs/String',
    callback,
    200
  );
}

// Publishing helpers
export function publishVelocity(linear: number, angular: number) {
  const twist: Twist = {
    linear: { x: linear, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: angular },
  };
  rosbridge.publishTo('/cmd_vel', 'geometry_msgs/Twist', twist);
}

export function publishCleaningCommand(command: 'start' | 'stop' | 'pause') {
  // Send to both cleaning systems for compatibility
  // Wall Follow Cleaner uses /cleaning_command
  rosbridge.publishTo('/cleaning_command', 'std_msgs/String', { data: command });
  
  // Advanced Autonomous Cleaner uses /autonomous/enable
  const enableAutonomous = command === 'start';
  rosbridge.publishTo('/autonomous/enable', 'std_msgs/Bool', { data: enableAutonomous });
}

export function publishEmergencyStop(stop: boolean) {
  rosbridge.publishTo('/emergency_stop', 'std_msgs/Bool', { data: stop });
}

export function publishRelayCommand(relay: number, state: boolean) {
  // relay: 1=Vacuum Pump, 2=Scrubber, 3=Sweeping Brush, 4=Water Pump
  const command = `${relay},${state ? 1 : 0}`;
  rosbridge.publishTo('/relay/command', 'std_msgs/String', { data: command });
}

export function subscribeRelayStatus(callback: (relay: number, state: boolean) => void) {
  return rosbridge.subscribeTo<{ data: string }>('/relay/status', 'std_msgs/String', (msg) => {
    const parts = msg.data.split(',');
    if (parts.length === 2) {
      const relay = parseInt(parts[0]);
      const state = parseInt(parts[1]) === 1;
      callback(relay, state);
    }
  });
}

// Subscribe to encoder data
export function subscribeEncoders(callback: (left: number, right: number) => void) {
  const encoderData = { left: 0, right: 0 };
  
  rosbridge.subscribeTo<{ data: number }>('/encoder/left', 'std_msgs/Int32', (msg) => {
    encoderData.left = msg.data;
    callback(encoderData.left, encoderData.right);
  }, 100);
  
  return rosbridge.subscribeTo<{ data: number }>('/encoder/right', 'std_msgs/Int32', (msg) => {
    encoderData.right = msg.data;
    callback(encoderData.left, encoderData.right);
  }, 100);
}

// Subscribe to water level sensors
export function subscribeWaterLevels(callback: (clean: number, dirty: number) => void) {
  const waterData = { clean: 0, dirty: 0 };
  
  rosbridge.subscribeTo<{ range: number }>('/water_level/clean', 'sensor_msgs/Range', (msg) => {
    waterData.clean = msg.range * 100; // Convert to percentage or cm
    callback(waterData.clean, waterData.dirty);
  }, 500);
  
  return rosbridge.subscribeTo<{ range: number }>('/water_level/dirty', 'sensor_msgs/Range', (msg) => {
    waterData.dirty = msg.range * 100; // Convert to percentage or cm
    callback(waterData.clean, waterData.dirty);
  }, 500);
}

export default rosbridge;
