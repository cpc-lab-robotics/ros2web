import * as cola from "webcola";

export interface Group extends cola.Group {
  id: string;
  name: string;
  
  selected: boolean;

  parent?: Group;
  component?: any;
}
