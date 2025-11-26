import { useEffect, useState } from 'react';
import { useRobotStore } from '../store/robotStore';
import rosbridge, {
  subscribeLaserScan,
  subscribeOdometry,
  subscribeMap,
  subscribeBattery,
  subscribeUltrasonic,
  subscribeIR,
  subscribeIMU,
  subscribeRelayStatus,
  subscribeEncoders,
  subscribeWaterLevels,
} from '../lib/rosbridge';
import { quaternionToEuler } from '../lib/utils';

export function useROS() {
  const [error, setError] = useState<string | null>(null);
  const setConnected = useRobotStore((state) => state.setConnected);
  const updatePose = useRobotStore((state) => state.updatePose);
  const updateVelocity = useRobotStore((state) => state.updateVelocity);
  const setEmergencyStop = useRobotStore((state) => state.setEmergencyStop);
  const updateBattery = useRobotStore((state) => state.updateBattery);
  const updateSensors = useRobotStore((state) => state.updateSensors);
  const updateMap = useRobotStore((state) => state.updateMap);
  const updateLidar = useRobotStore((state) => state.updateLidar);
  const updateCleaningSystem = useRobotStore((state) => state.updateCleaningSystem);
  const setHeading = useRobotStore((state) => state.setHeading);

  useEffect(() => {
    let mounted = true;

    // Connect to ROS bridge
    rosbridge
      .connect()
      .then(() => {
        if (!mounted) return;
        console.log('🤖 ROS connection established');
        setError(null);
      })
      .catch((err) => {
        if (!mounted) return;
        console.error('Failed to connect to ROS:', err);
        setError('Failed to connect to robot. Check if rosbridge is running.');
      });

    // Listen for connection changes
    const unsubscribe = rosbridge.onConnectionChange((connected) => {
      if (!mounted) return;
      setConnected(connected);
      if (!connected) {
        setError('Lost connection to robot');
      } else {
        setError(null);
      }
    });

    // Subscribe to topics
    const laserScanTopic = subscribeLaserScan((scan) => {
      if (!mounted) return;
      updateLidar(
        scan.ranges,
        scan.angle_min,
        scan.angle_max,
        scan.angle_increment
      );
      updateSensors({
        lidar: {
          ranges: scan.ranges,
          angle_min: scan.angle_min,
          angle_max: scan.angle_max,
          angle_increment: scan.angle_increment,
        },
      });
    });

    const odomTopic = subscribeOdometry((odom) => {
      if (!mounted) return;
      const { x, y } = odom.pose.pose.position;
      const { yaw } = quaternionToEuler(odom.pose.pose.orientation);
      updatePose(x, y, yaw);

      const lin = odom.twist?.twist?.linear?.x ?? 0;
      const ang = odom.twist?.twist?.angular?.z ?? 0;
      updateVelocity(lin, ang);
    });

    const mapTopic = subscribeMap((map) => {
      if (!mounted) return;
      console.log('🗺️  useROS: Map callback triggered', {
        width: map.info?.width,
        height: map.info?.height,
        hasData: !!map.data,
        dataLength: map.data?.length
      });
      updateMap({
        width: map.info.width,
        height: map.info.height,
        resolution: map.info.resolution,
        origin: {
          x: map.info.origin.position.x,
          y: map.info.origin.position.y,
        },
        data: map.data,
      });
      console.log('✅ Map state updated in store');
    });

    const batteryTopic = subscribeBattery((battery) => {
      if (!mounted) return;
      updateBattery(
        battery.voltage,
        battery.current,
        battery.percentage,
        battery.power_supply_status === 1 // 1 = charging
      );
    });

    const emergencyStopTopic = rosbridge.subscribeTo<{ data: boolean }>(
      '/emergency_stop',
      'std_msgs/Bool',
      (msg) => {
        if (!mounted) return;
        setEmergencyStop(msg.data);
      },
      200
    );

    const ultrasonicTopic = subscribeUltrasonic((sensors) => {
      if (!mounted) return;
      updateSensors({
        ultrasonic: {
          front: sensors.front,
          left_corner: sensors.left_corner,
          right_corner: sensors.right_corner,
          left_side: sensors.left_side,
          right_side: sensors.right_side,
        },
      });
    });

    const irTopic = subscribeIR((sensors) => {
      if (!mounted) return;
      updateSensors({
        ir: {
          obstacle_front_left: sensors.obstacle_front_left,
          obstacle_front_right: sensors.obstacle_front_right,
          obstacle_back_left: sensors.obstacle_back_left,
          obstacle_back_right: sensors.obstacle_back_right,
          cliff_front_left: sensors.cliff_front_left,
          cliff_front_right: sensors.cliff_front_right,
          cliff_back_left: sensors.cliff_back_left,
          cliff_back_right: sensors.cliff_back_right,
        },
      });
    });

    const imuTopic = subscribeIMU((imu) => {
      if (!mounted) return;
      const { yaw } = quaternionToEuler(imu.orientation);
      setHeading(yaw, 'imu');
      updateSensors({
        imu: {
          orientation: imu.orientation,
          angular_velocity: imu.angular_velocity,
          linear_acceleration: imu.linear_acceleration,
        },
      });
    });

    const relayStatusTopic = subscribeRelayStatus((relay, state) => {
      if (!mounted) return;
      // Map relay numbers to cleaning system names
      const systemMap: { [key: number]: 'vacuumPump' | 'scrubber' | 'sweepingBrush' | 'waterPump' } = {
        1: 'vacuumPump',
        2: 'scrubber',
        3: 'sweepingBrush',
        4: 'waterPump',
      };
      const system = systemMap[relay];
      if (system) {
        updateCleaningSystem(system, state);
      }
    });

    const encoderTopic = subscribeEncoders((left, right) => {
      if (!mounted) return;
      updateSensors({
        encoders: {
          left,
          right,
        },
      });
    });

    const waterLevelTopic = subscribeWaterLevels((clean, dirty) => {
      if (!mounted) return;
      updateSensors({
        water_levels: {
          clean,
          dirty,
        },
      });
    });

    // Cleanup
    return () => {
      mounted = false;
      unsubscribe();
      laserScanTopic?.unsubscribe();
      odomTopic?.unsubscribe();
      mapTopic?.unsubscribe();
      batteryTopic?.unsubscribe();
      ultrasonicTopic?.unsubscribe();
      irTopic?.unsubscribe();
      imuTopic?.unsubscribe();
      relayStatusTopic?.unsubscribe();
      encoderTopic?.unsubscribe();
      waterLevelTopic?.unsubscribe();
      emergencyStopTopic?.unsubscribe();
      rosbridge.disconnect();
    };
  }, [
    setConnected,
    updatePose,
    updateVelocity,
    updateBattery,
    updateSensors,
    updateMap,
    updateLidar,
    updateCleaningSystem,
    setHeading,
  ]);

  return { error };
}
