import { Data } from ".";
import { Topic } from "./topic";
import { Service } from "./service";
import { Action } from "./action";

export type Node = {
  name: string;
  namespace: string;
  fullName: string;
  interface?: NodeInterface;
} & Data;

export const nodeKeys = ["name", "namespace", "fullName"] as const;
export type NodeKeyType = typeof nodeKeys[number];
export function isNodeKey(key: string): key is NodeKeyType {
  return nodeKeys.includes(key as NodeKeyType);
}

export type NodeInterface = {
  nodeName: string;
  subscribers?: Topic[];
  publishers?: Topic[];
  serviceServers?: Service[];
  serviceClients?: Service[];
  actionServers?: Action[];
  actionClients?: Action[];
};
