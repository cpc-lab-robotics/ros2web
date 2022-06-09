import { Data } from ".";
import { Msg } from "./interface";

export type Topic = {
  name: string;
  type: Msg;
} & Data;
