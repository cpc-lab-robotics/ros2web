import React, { Dispatch } from "react";

import { WidgetEvent } from "./types";
import { Page } from "@/services/api";
import { LayoutClass } from "@/containers/layouts";


import WidgetComponent, { WidgetProps } from "./widget";

export default function createLayout(
  page: Page,
  webPackageName: string,
  handler: Dispatch<WidgetEvent>
) {
  const children: any[] = [];
  
  if (page.widgets) {
    for (const widget of page.widgets) {
      const widgetName = widget["name"];
      const widgetKey = widget['key'];
      const widgetId = widget['id'];
      const initState = widget['init_state'] || {};
      const props = widget["props"] || {};
      
      if (page.bind[widgetId]) {
        page.bind[widgetId].forEach((eventId: string) => {
          const [widgetId, eventType] = eventId.split(":");
          props[eventType] = (value: any) => {
            const we: WidgetEvent = {
              event: {
                widget_id: widgetId,
                type: eventType,
              },
              value: value,
            };
            handler(we);
          };
        });
      }
      const widgetProps: WidgetProps = {
        key: widgetKey,
        widgetKey,
        webPackageName,
        widgetName,
        initState,
        props,
      }
      children.push(React.createElement(WidgetComponent, widgetProps))
    }
  }

  if (page.style && LayoutClass[page.style]) {
    const props = {
      webPackageName,
      layout: page.layout
    };
    return React.createElement(LayoutClass[page.style], props, children);
  }
  return <></>;
}
