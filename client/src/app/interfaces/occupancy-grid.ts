import { MapOrientation } from "@app/interfaces/map-orientation";

export interface OccupancyGrid {
    data: Int8Array;
    height: number;
    width: number;
    resolution: number;
    origin: MapOrientation;
}