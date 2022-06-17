import * as cola from "webcola";

import { Group } from "./group";

export interface Node extends cola.Node {
  id: string;
  name: string;
  subGroup?: string;
  group?: string;

  pin: boolean;
  active: boolean;
  selected: boolean;

  nodeType: string;
  
  parent?: Group;
  component?: any;
}
