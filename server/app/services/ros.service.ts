process.env.ROS_DOMAIN_ID = '66';
import { Injectable, Logger, OnModuleInit } from '@nestjs/common';
import * as rclnodejs from 'rclnodejs';
import { LimoObject } from '@app/interfaces/LimoObject';
import { SocketService } from './socket/socket.service';

type RobotId = 'limo1' | 'limo2';
type Point2D = { x: number; y: number };

@Injectable()
export class RosService implements OnModuleInit {
  private readonly logger = new Logger(RosService.name);
  private limoList: LimoObject[];
  private mappingNode: rclnodejs.Node | undefined;
  private mappingSubscription: rclnodejs.Subscription | undefined;
  private poseNode: rclnodejs.Node | undefined;
  private poseSubscriptions: Partial<Record<RobotId, rclnodejs.Subscription>> = {};
  private points: Record<RobotId, Point2D[]> = { limo1: [], limo2: [] };
  private isActing: Record<RobotId, boolean> = { limo1: false, limo2: false };
  private navClient1: any;
  private navClient2: any;

  constructor(private socketService: SocketService) {}

  async onModuleInit() {
    await rclnodejs.init();
    const nodeLimo1 = new rclnodejs.Node('identify_client_backend', 'limo1');
    const nodeLimo2 = new rclnodejs.Node('identify_client_backend', 'limo2');
    const Trigger = (rclnodejs.require('std_srvs') as any).srv.Trigger;
    const clientLimo1 = nodeLimo1.createClient(Trigger, 'identify_robot');
    const clientLimo2 = nodeLimo2.createClient(Trigger, 'identify_robot');
    this.limoList = [
      { node: nodeLimo1, identifyClient: clientLimo1 },
      { node: nodeLimo2, identifyClient: clientLimo2 }
    ];
    this.navClient1 = new rclnodejs.ActionClient(nodeLimo1, 'nav2_msgs/action/FollowWaypoints', 'follow_waypoints');
    this.navClient2 = new rclnodejs.ActionClient(nodeLimo2, 'nav2_msgs/action/FollowWaypoints', 'follow_waypoints');
    nodeLimo1.spin();
    nodeLimo2.spin();
    this.setupMappingListner();
    this.setupPoseListners();
    this.logger.log('ROS2 client prêt !');
  }

  private setupMappingListner() {
    this.mappingNode = new rclnodejs.Node('mapping_listener_backend');
    this.mappingSubscription = this.mappingNode.createSubscription(
      'nav_msgs/msg/OccupancyGrid',
      '/limo1/map',
      (msg) => {
        this.socketService.sendMapToAllSockets(msg);
      }
    );
    this.mappingNode.spin();
    console.log('ROS2 subscriber started');
  }

  private setupPoseListners() {
    this.poseNode = new rclnodejs.Node('pose_listener_backend');
    this.poseSubscriptions.limo1 = this.poseNode.createSubscription(
      'geometry_msgs/msg/PoseWithCovarianceStamped',
      '/limo1/pose',
      (msg) => {
        const payload = this.extractPosePayload(msg);
        console.log('Received pose data for limo1:', payload);
        this.socketService.sendPoseToAllSockets('limo1', payload);
      }
    );
    this.poseSubscriptions.limo2 = this.poseNode.createSubscription(
      'geometry_msgs/msg/PoseWithCovarianceStamped',
      '/limo2/pose',
      (msg) => {
        const payload = this.extractPosePayload(msg);
        console.log('Received pose data for limo2:', payload);
        this.socketService.sendPoseToAllSockets('limo2', payload);
      }
    );
    this.poseNode.spin();

    console.log('ROS2 pose subscribers started');
  }

  async identifyRobot(id: number): Promise<{ success: boolean; message: string }> {
    const obj = this.limoList[id - 1];
    if (!obj.identifyClient) {
      this.logger.error('Client ROS2 non initialisé');
      return { success: false, message: 'ROS2 non initialisé' };
    }

    return new Promise((resolve) => {
      const request = new ((rclnodejs.require('std_srvs') as any).srv.Trigger.Request)();
      obj.identifyClient.sendRequest(request, (response) => {
        if (response) resolve({ success: response.success, message: response.message });
        else resolve({ success: false, message: 'Échec de l’appel ROS2' });
      });
    });
  }

  async handlePoints(payload: { robot: RobotId; points: Point2D[] }) {
    if (!payload.points?.length) return;
    this.points[payload.robot].push(...payload.points);
    if (!this.isActing[payload.robot]) {
      this.processPointQueue(payload.robot).catch((err) =>
        this.logger.error(`Failed to process queue for ${payload.robot}`, err.stack),
      );
    }
  }

  private async processPointQueue(robot: RobotId) {
    const queue = this.points[robot];
    if (!queue.length) {
      this.isActing[robot] = false;
      return;
    }
    this.isActing[robot] = true;
    const waypoints = queue.splice(0, queue.length);
    try {
      await this.dispatchWaypoints(robot, waypoints);
    } catch (err) {
      this.logger.error(`Error sending waypoints for ${robot}`, (err as Error).stack);
    } finally {
      if (this.points[robot].length) {
        await this.processPointQueue(robot);
      } else {
        this.isActing[robot] = false;
      }
    }
  }

  private async dispatchWaypoints(robot: RobotId, waypoints: Point2D[]) {
    const client = robot === 'limo1' ? this.navClient1 : this.navClient2;
    if (!client) throw new Error(`Action client for ${robot} not ready`);
    await (client as any).waitForServer?.();
    const goal = {
      poses: waypoints.map((point) => ({
        header: {
          stamp: this.buildRosTime(),
          frame_id: `${robot}/map`,
        },
        pose: {
          position: { x: point.x, y: point.y, z: 0 },
          orientation: { x: 0, y: 0, z: 0, w: 1 },
        },
      })),
      behavior_tree: '',
    };
    console.log(goal);
    console.log(goal.poses)
    const goalHandle = await client.sendGoal(goal, (feedback) =>
      this.logger.debug(
        `[${robot}] current waypoint index: ${feedback?.current_waypoint}, current pose: ${feedback?.current_pose?.pose}`,
      ),
    );
    const result = await goalHandle.getResult();
    this.logger.log(`[${robot}] waypoint result: ${result?.status}`);
  }

  private buildRosTime() {
    const now = Date.now();
    return {
      sec: Math.floor(now / 1000),
      nanosec: (now % 1000) * 1e6,
    };
  }

  private extractPosePayload(msg: any) {
    if (msg?.pose?.pose) {
      return {
        header: msg.header,
        pose: msg.pose.pose,
      };
    }
    return msg;
  }
}
