import {Element} from "../types";

export interface WebPackage {
  name: string;
  packageName: string;
  active: boolean;
  ui: UIData;
}

export interface UIData {
  element: Element;
  bind: { [widgetId: string]: string[] };
}

