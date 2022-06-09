export type Param = {
  param?: Param;
  name?: string;
  value?: string[] | number[] | string | number ;
  fromPath?: string;
  
  groupName?: string;
}
