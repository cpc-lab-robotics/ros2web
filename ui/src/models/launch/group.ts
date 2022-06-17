import { Executable } from "./executable";
import { Node } from "./node";
import { Include } from "./include";
import { Let } from "./let";
import { SetEnv } from "./set-env";
import { UnsetEnv } from "./unset-env";


export type Group = {
  executable?: Executable[];
  node?: Node[];
  include?: Include[];
  let?: Let[];
  setEnv?: SetEnv[];
  unsetEnv?: UnsetEnv[];
  scoped?: boolean;
  ifCondition?: string;
  unlessCondition?: string;
  group?: Group[];
}
