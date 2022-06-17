import { Env } from "./env";
import { Param } from "./param";
import { Remap } from "./remap";

export * from "./param";
export * from "./remap";

export type Arg = {
  name: string;
  value?: string;
  default?: string;
  description?: string;
}

export type Node = {
  pkg: string;
  exec: string;
  name?: string;
  rosArgs?: string;
  args?: string;
  namespace?: string;
  launchPrefix?: string;
  output?: "log" | "screen";
  envs?: Env[];
  params?: Param[];
  remaps?: Remap[];
  ifCondition?: string;
  unlessCondition?: string;

  id?: string;
  debugLevel?: "INFO" | "DEBUG";
}