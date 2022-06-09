import React, { useEffect, useMemo, useRef, useState } from "react";
import { useProps } from "@/services/api";
import { useGlobalState } from "@/hooks";
import NodeGraph, { NodeGraphProps as _NodeGraphProps } from "@/components/Graph";
import {
  Size
} from "@/components/types";
import { WidgetProps, WidgetEvent } from "./types";
import { GraphNode, GraphLink} from "@/components/Graph/types";
import { Data } from "@/models";

import Box from "@mui/material/Box";


function areEqual(prevProps: any, nextProps: any) {
  for (const key of Object.keys(nextProps)) {
    if (key in prevProps) {
      if (prevProps[key] !== nextProps[key]) {
        return false;
      }
    } else {
      return false;
    }
  }
  return true;
}

const GraphMemo = React.memo((props: _NodeGraphProps) => {
  return <NodeGraph {...props} />;
}, areEqual);

export type NodeGraphProps = {
  selected_id?: string;
  on_selected?: (event: WidgetEvent) => void;
  on_did_update?: (event: WidgetEvent) => void;
};

type Props = NodeGraphProps & WidgetProps;

export default function GraphWidget(props: Props) {
  const {
    webPackageName,
    initState,
    bindStateKey,
    setState,
    dataAdaptor,
    selected_id,
    on_selected,
    on_did_update,
  } = props;

  let graphNodes: GraphNode[] = [];
  let graphLinks: GraphLink[] = [];

  if (dataAdaptor) {
    // const data = dataAdaptor.useGraphData();
    // graphNodes = data[0];
    // graphLinks = data[1];
    graphNodes = dataAdaptor.useGraphData();
  }

  let updateProps: Record<string, any> = {}
  if (webPackageName && initState && bindStateKey) {
    const bindProps: Record<string, string> = {};
    for (const prop of ["selected_id"]){
      const stateKey = bindStateKey[prop];
      if(stateKey) {
        bindProps[stateKey] = prop;
      }
    }
    updateProps = useProps(
      webPackageName,
      initState,
      bindProps
    );
  }

  const ref = useRef<any>(null);
  const [size, setSize] = useState<Size>({ width: 0, height: 0 });
  const [resizeDate] = useGlobalState<Date>(["resizeEvent"]);
  useEffect(() => {
    if (ref.current) {
      const width = ref.current.offsetWidth;
      let height;
      height = ref.current.offsetHeight;

      // if (cardStyle) {
      //   height = Math.max(200, width);
      // } else {
      // }

      setSize({ width, height });
    }
  }, [resizeDate]);

  const layoutDidUpdateHandler = () => {
    
    if (on_did_update) {
      const event: WidgetEvent = {
        event: {
          widget_id: "__id__",
          type: "on_did_update",
        },
      };
      on_did_update(event);
    }
  };
  
  const selectedHandler = (graphNode:GraphNode|undefined) => {
    // let data: Data | undefined = undefined;
    // if (dataAdaptor && graphNode) {
    //   data = dataAdaptor.graphDataToData(graphNode);
    // }
    if (on_selected) {
      const event: WidgetEvent = {
        event: {
          widget_id: "__id__",
          type: "on_selected",
        },
        value: (graphNode) ? graphNode.id : null,
      };
      on_selected(event);
    }
  };

  const graphProps: _NodeGraphProps = {
    graphNodes,
    size,
    onLayoutDidUpdate: layoutDidUpdateHandler,
    onSelected: selectedHandler,
    selectedId: updateProps["selected_id"] || selected_id,
  };

  return (
    <Box
      ref={ref}
      sx={{ p: 0 }}
      style={{
        width: "100%",
        height: "100%",
      }}
    >
      <GraphMemo {...graphProps} />
    </Box>
  );
}
