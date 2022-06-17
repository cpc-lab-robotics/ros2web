import { Data } from ".";
import {Act} from "./interface";

export type Action = {
  name: string;
  type: Act
} & Data;
