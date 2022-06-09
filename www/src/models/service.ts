import { Data } from ".";
import { Srv } from "./interface";

export type Service = {
  name: string;
  type: Srv;
} & Data;
