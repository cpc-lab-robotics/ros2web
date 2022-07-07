import React, { Dispatch } from "react";
import { useQueryClient } from "react-query";
import { WidgetEvent } from "@/containers/page/types";

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

export const useWebSocket = (
  webPackageName: string
): [
  Dispatch<WidgetEvent>
] => {
  const queryClient = useQueryClient();
  const websocket = React.useRef<WebSocket>();
  
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

  const emit = (event: WidgetEvent) => {
    const ev: EmitEvent = {
      operation: "emit",
      webPackageName,
      widgetEvent: event,
    };
    websocket.current?.send(JSON.stringify(ev));
  };

  return [emit];
};
