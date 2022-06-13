import React, { Dispatch } from "react";

import { layouts } from "@/containers/layouts";
import { WidgetEvent } from "@/containers/widgets/types";
import { Page } from "@/services/api";

import createWidget from "./create-widget";
import { extractStateKey } from "@/utils/extract";

export default function createLayout(
  page: Page,
  webPackageName: string,
  state: Record<string, any>,
  setState: React.Dispatch<Record<string, any>>,
  handler: Dispatch<WidgetEvent>
) {
  const children: any[] = [];

  if (page.ui.widgets) {
    for (const widget of page.ui.widgets) {
      const widget_el = createWidget(
        widget,
        webPackageName,
        state,
        setState,
        handler,
        page.bind
      );
      if (widget_el) {
        children.push(widget_el);
      }
    }
  }

  if (page.ui.layout && typeof page.ui.layout === "object") {
    const [layoutName, layoutProps] =
      Object.entries(page.ui.layout).length > 0
        ? Object.entries(page.ui.layout)[0]
        : ["", null];
    
    if (typeof layouts[layoutName] !== "undefined") {
      const props: Record<string, any> = {};
      const initState: Record<string, any> = {};
      const bindStateKey: Record<string, string> = {};
      if (layoutProps && typeof layoutProps === "object") {
        for (const [prop, prop_value] of Object.entries(layoutProps)) {
          const stateKey = extractStateKey(prop_value);
          if (stateKey !== null) {
            if (!!state) {
              initState[stateKey] = state[stateKey];
              bindStateKey[prop] = stateKey;
            }
          } else {
            if (prop === "id") {
              if (page.bind[prop_value]) {
                page.bind[prop_value].forEach((eventId: string) => {
                  const [widgetId, eventType] = eventId.split(":");
                  props[eventType] = (event: WidgetEvent) => {
                    event.event.widget_id = widgetId;
                    handler(event);
                  };
                });
              }
            } else {
              props[prop] = prop_value;
            }
          }
        }
      }
      props["webPackageName"] = webPackageName;
      props["initState"] = initState;
      props["bindStateKey"] = bindStateKey;
      props["setState"] = setState;

      return React.createElement(layouts[layoutName], props, children);
    }
  }
  return <></>;
}
