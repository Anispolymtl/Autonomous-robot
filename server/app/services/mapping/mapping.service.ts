import { Injectable, Logger } from "@nestjs/common";
import * as rclnodejs from 'rclnodejs';
import { SocketService } from "../socket/socket.service";

type RobotId = 'limo1' | 'limo2';

@Injectable()
export class MappingSerivce {
private mappingNode: rclnodejs.Node | undefined;
private poseNode: rclnodejs.Node | undefined;
private readonly logger = new Logger(MappingSerivce.name);

constructor (
    private socketService: SocketService
) {}

initialiseMappingService() {
    this.setupMappingListner('limo1');
    this.setupMappingListner('limo2');
    this.setupMergedMapListner();
    this.setupPoseListners('limo1');
    this.setupPoseListners('limo2');
    console.log('mapping started')
}

private setupMappingListner(robotId: RobotId) {
    this.mappingNode = new rclnodejs.Node(`mapping_listener_backend`, robotId);
    this.mappingNode.createSubscription(
      'nav_msgs/msg/OccupancyGrid',
      `map`,
      (msg) => {
        this.socketService.sendMapToAllSockets(msg, robotId);
      }
    );
    this.mappingNode.spin();
    console.log('ROS2 subscriber started');
  }

  private setupPoseListners(robotId: RobotId) {
    this.poseNode = new rclnodejs.Node('pose_listener_backend', robotId);
    this.poseNode.createSubscription(
      'geometry_msgs/msg/PoseStamped',
      `current_pose`,
      (msg) => {
        this.socketService.sendPoseToAllSockets(robotId, msg);
      }
    );
    this.poseNode.spin();
  }

  private setupMergedMapListner() {
    console.log('setup merged map listner')
    this.mappingNode = new rclnodejs.Node('merged_mapping_listner_backend');
    this.mappingNode.createSubscription(
      'nav_msgs/msg/OccupancyGrid',
      `/merged_map`,
      (msg) => {
        console.log(msg);
        this.socketService.sendMergedMapToAllSockets(msg);
      }
    );

    this.mappingNode.spin();
  }
}
