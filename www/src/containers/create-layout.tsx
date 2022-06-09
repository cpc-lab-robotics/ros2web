import React, { Dispatch } from "react";
import { Element, isBoolean } from "./types";
import StringElement, { StringProps } from "./string";

import { layouts } from "@/containers/layouts";
import { WidgetEvent } from "@/containers/widgets/types";

import createWidget from "./create-widget";
import { extractStateKey } from "@/utils/extract";


export default function createLayout(
  webPackageName: string,
  element: Element | string,
  bind: { [widgetId: string]: string[] },
  state: Record<string, any>,
  setState: React.Dispatch<Record<string, any>>,
  handler: Dispatch<WidgetEvent>,
): any {
  if (element === undefined || element === null) return;

  if (typeof element === "string") {
    const stateKey = extractStateKey(element);
    if (!!state && stateKey) {
      const props: StringProps = {
        webPackageName,
        stateKey,
        value: state[stateKey],
      };
      return React.createElement(StringElement, props);
    } else {
      return element;
    }
  } else if (typeof layouts[element.name] !== "undefined") {

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
        if (prop === "id") {
          if (bind[v]) {
            bind[v].forEach((eventId) => {
              const [widgetId, eventType] = eventId.split(":");
              props[eventType] = (event: WidgetEvent) => {
                event.event.widget_id = widgetId;
                handler(event);
              };
            });
          }
        }
      });
    }

    let children: any = null;
    if (element.children) {
      if (Array.isArray(element.children)) {
        children = element.children.map((el: Element) =>
          createWidget(
            webPackageName,
            el,
            bind,
            state,
            setState,
            handler
          )
        );
      } else {
        // children = createWidget(
        //   webPackageName,
        //   element.children,
        //   bind,
        //   state,
        //   setState,
        //   handler
        // )
      }
    }

    props["webPackageName"] = webPackageName;
    props["initState"] = initState;
    props["bindStateKey"] = bindStateKey;
    props["setState"] = setState;
    return React.createElement(layouts[element.name], props, children);
  } else {
    return;
  }
}
