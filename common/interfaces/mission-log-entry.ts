export type MissionLogCategory = 'Command' | 'Sensor';

export type MissionLogDetailValue = string | number | boolean | null | undefined;

export interface MissionLogDetails {
    [key: string]: MissionLogDetailValue;
}

export interface MissionLogEntry {
    timestamp: string;
    robot: string;
    category: MissionLogCategory;
    action: string;
    details: MissionLogDetails;
}
