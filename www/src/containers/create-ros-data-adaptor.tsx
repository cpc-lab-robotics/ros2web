import React from "react";
import { Element, isBoolean } from "./types";
import { dataAdaptors } from "@/containers/data/ros";
import ROSDataAdaptor from "@/containers/data/ros/base";

import { extractStateKey } from "@/utils/extract";

export default function createSubDataAdaptor(
  webPackageName: string,
  element: Element,
  state: Record<string, any>,
  setState: React.Dispatch<Record<string, any>>
): ROSDataAdaptor | undefined {
  if (
    element &&
    element.name &&
    typeof dataAdaptors[element.name] !== "undefined"
  ) {
    const props: Record<string, any> = {};
    const initState: Record<string, any> = {};
    const bindStateKey: Record<string, string> = {};

    if (element.props) {
      Object.entries(element.props).forEach(([prop, v]) => {
        const stateKey = extractStateKey(v);
        if (stateKey !== null) {
          if (!!state) {
            initState[stateKey] = state[stateKey];
            bindStateKey[prop] = stateKey;
          }
        } else {
          if (isNaN(v)) {
            if (isBoolean(v)) {
              props[prop] = Boolean(v.toLowerCase());
            } else {
              props[prop] = v;
            }
          } else {
            props[prop] = Number(v);
          }
        }
      });
    }
    props["webPackageName"] = webPackageName;
    props["initState"] = initState;
    props["bindStateKey"] = bindStateKey;
    props["setState"] = setState;
    const class_object = dataAdaptors[element.name];
    const adaptor: ROSDataAdaptor = new class_object(props);
    return adaptor;
  }

  return;
}
