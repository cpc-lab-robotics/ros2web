import React, { useEffect, useState, useMemo } from "react";

import { extractStateKey } from "@/utils/extract";
import { useStateKey } from "@/services/api/use-state-key";
import { loadWidget } from "@/services/api/load-widget";

import Box from "@mui/material/Box";
import CircularProgress from "@mui/material/CircularProgress";

export type WidgetProps = {
  key: string;
  widgetKey: string;
  webPackageName: string;
  widgetName: string;
  initState: Record<string, any>;
  props: Record<string, any>;
};

function Loading() {
  return (
    <Box
      sx={{
        width: "100%",
        height: "100%",
        display: "flex",
        justifyContent: "center",
        alignItems: "center",
      }}
    >
      <CircularProgress />
    </Box>
  );
}

export default function Widget(widgetProps: WidgetProps) {
  const { webPackageName, widgetName, initState, props } = widgetProps;
  
  const Component = useMemo(() => {
    const [plugin, widget] = widgetName.split(".");
    return React.lazy(loadWidget(plugin, widget));
  }, [widgetName]);

  const state = useStateKey(webPackageName, initState);
  const newProps = updateProps(props, state);
  
  return (
    <React.Suspense fallback={<Loading />}>
      <Component {...newProps} />
    </React.Suspense>
  );
}

function updateProps(data: any, state: Record<string, any>): any {
  if (typeof data === "string") {
    const stateKey = extractStateKey(data);
    if (stateKey !== null) {
      return state[stateKey];
    }
  } else if (Array.isArray(data)) {
    return data.map((value) => updateProps(value, state));
  } else if (data.constructor == Object) {
    const props: Record<string, any> = {};
    for (const [prop, propValue] of Object.entries<any>(data)) {
      props[prop] = updateProps(propValue, state);
    }
    return props;
  }
  return data;
}
