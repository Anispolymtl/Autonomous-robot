export interface PoseData {
    header: { frame_id: string; stamp?: { sec: number; nanosec: number } };
    pose: {
        position: { x: number; y: number; z: number };
        orientation: { x: number; y: number; z: number; w: number };
    };
}