import { Data } from ".";

export type Field = {
  type: string;
  name: string;
  nested?: Field[];
};

export type Msg = {
  name: string;
  packageName: string;
  fields?: Field[];
} & Data;

export type Srv = {
  name: string;
  packageName: string;
  request?: Field[];
  response?: Field[];
} & Data;

export type Act = {
  name: string;
  packageName: string;
  goal?: Field[];
  result?: Field[];
  feedback?: Field[];
} & Data;
