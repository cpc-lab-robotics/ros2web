import * as cola from "webcola";
import { Node } from "./node";

export interface Link extends cola.Link<Node> {
  id: string;
  name: string;
  type: string;
  selected: boolean;
  component?: any;
}