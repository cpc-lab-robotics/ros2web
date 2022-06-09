export type GraphLink = {
  name: string;
  type: string;
  nodeType: string;
  active?: boolean;
}

export type GraphNode = {
  id: string;
  nodeType: string;
  name: string;
  x?: number;
  y?: number;
  group?: string;
  subGroup?: string;
  inbounds?: GraphLink[];
  outbounds?: GraphLink[];
  pin?: boolean;
  active?: boolean;
};

export type GraphGroup = {
  groupName: string;
  subGroupName?: string;
};

