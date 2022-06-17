export type Arg = {
  name: string;
  value: string;
}

export type Include = {
  file: string;
  arg?: string;
  ifCondition?: string;
  unlessCondition?: string;
}