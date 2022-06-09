import { Env } from "./env";

export type Executable = {
  cmd: string;
  cwd?: string;
  name?: string;
  args?: string;
  shell?: boolean;
  launch_prefix?: string;
  output?: "log" | "screen";
  env?: Env[];
  ifCondition?: string;
  unlessCondition?: string;
}
