import * as rclnodejs from 'rclnodejs';

export interface LimoObject {
    node: rclnodejs.Node;
    identifyClient: any;
    returnClient: any;
}