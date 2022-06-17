import React from "react";
import { DataContainerClass } from "@/containers/data";
import DataContainer, { DataContainerProps } from "@/containers/data/container";
import { extractStateKey } from "@/utils/extract";

export default function createDataContainers(
  containers: any[],
  webPackageName: string,
  state: Record<string, any>,
  setState: React.Dispatch<Record<string, any>>
): DataContainer[] {
  const dataContainers: DataContainer[] = [];

  for (const container of containers) {
    const [containerName, containerProps] =
      Object.entries(container).length > 0
        ? Object.entries(container)[0]
        : ["", null];

    if (typeof DataContainerClass[containerName] !== "undefined") {
      const props: Record<string, any> = {};
      const initState: Record<string, any> = {};
      const bindStateKey: Record<string, string> = {};

      for (const [prop, propValue] of Object.entries(
        containerProps as Object
      )) {
        const stateKey = extractStateKey(propValue);
        if (stateKey !== null) {
          if (!!state) {
            initState[stateKey] = state[stateKey];
            bindStateKey[prop] = stateKey;
          }
        } else {
          props[prop] = propValue;
        }
      }
      
      const dataContainerProps: DataContainerProps = {
        webPackageName,
        initState,
        bindStateKey,
        setState,
      };
      const container: DataContainer = new DataContainerClass[containerName]({
        ...props,
        ...dataContainerProps,
      });
      dataContainers.push(container);
    }
  }
  return dataContainers;
}
