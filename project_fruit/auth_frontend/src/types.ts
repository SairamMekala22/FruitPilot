export interface Waypoint {
  lat: number;
  lng: number;
}

export interface FruitType {
  id: string;
  name: string;
  color: string;
}

export interface ModelType {
  id: string;
  name: string;
  description: string;
}

export interface Mission {
  id: string;
  name: string;
  date: string;
  location: string;
  fruitType: string;
  modelType: string;
  status: 'pending' | 'in-progress' | 'completed' | 'failed';
  detectedFruits: number;
  duration: string;
  waypoints: Waypoint[];
  coverageArea: string;
}
export interface Waypoint {
  lat: number;
  lng: number;
}
