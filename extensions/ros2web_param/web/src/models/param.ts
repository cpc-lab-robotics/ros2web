import { Data } from "./index";

export type Param = {
  nodeName: string;
  name: string;
  value: any;
  descriptor?: Descriptor;
} & Data;

export type Range = {
  fromValue: number;
  toValue: number;
  step?: number;
};

export const PARAMETER = {
  NOT_SET: "PARAMETER_NOT_SET",
  BOOL: "PARAMETER_BOOL",
  INTEGER: "PARAMETER_INTEGER",
  DOUBLE: "PARAMETER_DOUBLE",
  STRING: "PARAMETER_STRING",
  BYTE_ARRAY: "PARAMETER_BYTE_ARRAY",
  BOOL_ARRAY: "PARAMETER_BOOL_ARRAY",
  INTEGER_ARRAY: "PARAMETER_INTEGER_ARRAY",
  DOUBLE_ARRAY: "PARAMETER_DOUBLE_ARRAY",
  STRING_ARRAY: "PARAMETER_STRING_ARRAY",
} as const;
type ParamType = typeof PARAMETER[keyof typeof PARAMETER];

export type Descriptor = {
  name: string;
  type: ParamType;
  description: string;

  additionalConstraints?: string;
  readOnly?: boolean;
  dynamicTyping?: boolean;
  floatingPointRange?: Range;
  integerRange?: Range;
};