import { Executable } from "./executable";
import { Node } from "./node";
import { Include } from "./include";
import { Let } from "./let";
import { SetEnv } from "./set-env";
import { UnsetEnv } from "./unset-env";
import { Group } from "./group";


export type Arg = {
  name: string;
  value?: string;
  default?: string;
  description?: string;
}


export type Launch = {
  version: string;
  arg?: Arg[];
  let?: Let[];
  node?: Node[];
  executable?: Executable[];
  include?: Include[];
  group?: Group[];
  setEnv?: SetEnv[];
  unsetEnv?: UnsetEnv[];
}


// export * from "./env";


