import React, { Dispatch, SetStateAction } from "react";

import { useQueryClient, useQuery, useQueries } from "react-query";
import { WidgetEvent } from "@/containers/widgets/types";

import { getState, getPage, Page } from "./ros2web";

type InvalidateEvent = {
  operation: "invalidate";
  webPackageName: string;
};

type UpdateEvent = {
  operation: "update";
  webPackageName: string;
  payload: Record<string, any>;
};

type EmitEvent = {
  operation: "emit";
  webPackageName: string;
  widgetEvent: WidgetEvent;
};

type WebSocketEvent = InvalidateEvent | UpdateEvent | EmitEvent;

export const useWebPackageState = (
  webPackageName: string, 
  path: string
): [
  Page | undefined,
  Record<string, any> | undefined,
  Dispatch<Record<string, any>>,
  Dispatch<WidgetEvent>
] => {
  const queryClient = useQueryClient();
  const websocket = React.useRef<WebSocket>();

  const results = useQueries([
    {
      queryKey: [webPackageName, "page"],
      queryFn: () => getPage(webPackageName, path),
    },
    {
      queryKey: [webPackageName, "state"],
      queryFn: () => getState(webPackageName),
    },
  ]);

  const { data: page } = results[0];
  const { data: state } = results[1];

  React.useEffect(() => {
    const url = `//${window.location.host}/subscription/state`;
    // const url = `//localhost:8080/subscription/state`;

    websocket.current = new WebSocket(
      (window.location.protocol === "https:" ? "wss:" : "ws:") + url
    );
    websocket.current.onmessage = (event) => {
      // console.log("received event", event);
      const data: WebSocketEvent = JSON.parse(event.data);
      switch (data.operation) {
        case "invalidate":
          queryClient.invalidateQueries(
            [data.webPackageName, "state"].filter(Boolean)
          );
          break;
        case "update":
          Object.entries(data.payload).forEach(([key, value]) => {
            queryClient.setQueriesData(
              [data.webPackageName, "state", key],
              value
            );
          });
          break;
      }
    };
    websocket.current.onopen = () => {
      console.log("connected");
    };
    websocket.current.onerror = (event) => {
      console.log("websocket event", event);
    };
    return () => {
      websocket.current?.close();
    };
  }, [queryClient]);

  const setState = (state: Record<string, any>): void => {
    for (const [key, value] of Object.entries(state)) {
      const queryKey = [webPackageName, "state", key];
      queryClient.setQueryData<Record<string, any>>(queryKey, value);
    }
    // const event: UpdateEvent = {
    //   operation: "update",
    //   webPackageName,
    //   payload: newValue,
    // };
    // websocket.current?.send(JSON.stringify(event));
  };

  const emit = (event: WidgetEvent) => {
    const ev: EmitEvent = {
      operation: "emit",
      webPackageName,
      widgetEvent: event,
    };
    websocket.current?.send(JSON.stringify(ev));
  };

  return [page, state, setState, emit];
};
