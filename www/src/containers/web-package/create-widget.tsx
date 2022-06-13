import React, { Dispatch } from "react";

import { extractStateKey } from "@/utils/extract";

import { WidgetClass } from "../widgets";
import { WidgetEvent } from "../widgets/types";
import DataAdaptor, { DataAdaptorProps } from "../data/adaptor";
import DataContainer from "../data/container";
import createDataContainers from "./create-data-containers";

export default function createWidget(
  widget: Record<string, any>,
  webPackageName: string,
  state: Record<string, any>,
  setState: React.Dispatch<Record<string, any>>,
  handler: Dispatch<WidgetEvent>,
  bind: { [widgetId: string]: string[] }
) {
  const [widgetName, widgetProps] =
    Object.entries(widget).length > 0 ? Object.entries(widget)[0] : ["", null];

  if (typeof WidgetClass[widgetName] !== "undefined") {
    const props: Record<string, any> = {};
    const initState: Record<string, any> = {};
    const bindStateKey: Record<string, string> = {};
    let dataContainers: DataContainer[] = [];
    if (widgetProps && typeof widgetProps === "object") {
      for (const [prop, propValue] of Object.entries<any>(widgetProps)) {
        if (prop === "data_adaptor") {
          dataContainers = createDataContainers(
            propValue,
            webPackageName,
            state,
            setState
          );
        } else {
          const stateKey = extractStateKey(propValue);
          if (stateKey !== null) {
            if (!!state) {
              initState[stateKey] = state[stateKey];
              bindStateKey[prop] = stateKey;
            }
          } else {
            if (prop === "id") {
              if (bind[propValue]) {
                bind[propValue].forEach((eventId: string) => {
                  const [widgetId, eventType] = eventId.split(":");
                  props[eventType] = (event: WidgetEvent) => {
                    event.event.widget_id = widgetId;
                    handler(event);
                  };
                });
              }
            } else {
              props[prop] = propValue;
            }
          }
        }
      }
    }

    const dataAdaptorProps: DataAdaptorProps = {
      webPackageName,
      initState,
      bindStateKey,
      setState,
      dataContainers,
    };

    const children: any[] = [];
    const dataAdaptor = new DataAdaptor(dataAdaptorProps);
    props["dataAdaptor"] = dataAdaptor;
    
    if (widgetName === "Stack") {
      const widgets = props["widgets"];
      for (const _widget of widgets) {
        const widget_el = createWidget(
          _widget,
          webPackageName,
          state,
          setState,
          handler,
          bind
        );
        if (widget_el) {
          children.push(widget_el);
        }
      }
    }

    return React.createElement(WidgetClass[widgetName], props, children);
  }

  return <></>;
}
