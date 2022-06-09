export type Element = {
  name: string;
  props?: { [prop: string]: any };
  children?: string | Element[];
}

export function isBoolean(value: string): boolean {
  return value.toLowerCase() === "true" || value.toLowerCase() === "false";
}
