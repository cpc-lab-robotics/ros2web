import { Data } from ".";
import { Srv } from "./interface";

export type Service = {
  name: string;
  type: string;
  requestValue?: any;
  responseValue?: any;
  descriptor?: Srv;
} & Data;
