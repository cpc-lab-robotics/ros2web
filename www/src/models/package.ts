import {Data} from ".";

export type Package = {
  name: string;
  executables: string[];
} & Data;

export type PackageManifest = {
  filename: string;
  packageFormat: number;
  
  // Required Tags
  name: string;
  version: string;
  description: string;
  maintainers: Person[];
  licenses: string[];

  // Optional Tags
  urls?: Url[];
  authors?: Person[];

  buildDepends?: Dependency[];
  buildExportDepends?: Dependency[];
  buildtoolDepends?: Dependency[];
  buildtoolExportDepends?: Dependency[];
  execNameDepends?: Dependency[];
  depends?: Dependency[];
  docDepends?: Dependency[];
  testDepends?: Dependency[];
  conflicts?: Dependency[];
  replaces?: Dependency[];
  groupDepends?: Dependency[];
  memberOfGroups?: Dependency[];

  exports?: Export[];
} & Data;

export type Dependency = {
  name: string;
  versionLt?: string;
  versionLte?: string;
  versionEq?: string;
  versionGte?: string;
  versionGt?: string;
  condition?: string;
} 

export type Person = {
  name: string;
  email?: string;
}

export type Url = {
  url: string;
  type?: string;
} 

export type Export = {
  tagname: string;
  attributes: { [key: string]: string };
  content: string;
}
