import React, { useState, useCallback } from "react";

import SVG from "../svg/SVG";
import Grid, { GridProps } from "../svg/Grid";
import ZoomContainer from "../svg/ZoomContainer";
import GraphLayout from "./layout";
import { GraphNode, GraphLink, GraphGroup } from "./types";
import { Size, Transform } from "../types";

const GRID_WIDTH = 1000;
const GRID_HEIGHT = 1000;
const GRID_COUNT = 100;

const GRID_OFFSET_X = (-1 * GRID_WIDTH) / 2;
const GRID_OFFSET_Y = (-1 * GRID_HEIGHT) / 2;

const OFFSET_X = 0;
const OFFSET_Y = 0;

export type NodeGraphProps = {
  size: Size;
  graphNodes?: GraphNode[];
  onSelected?: (graphNode: GraphNode | undefined) => void;
  onLayoutDidUpdate?: () => void;
  selectedId?: string;
};

export default function NodeGraph(props: NodeGraphProps) {
  const { size, graphNodes, onSelected, onLayoutDidUpdate, selectedId } = props;
  
  const [zoomTransform, setZoomTransform] = useState<Transform>({
    x: OFFSET_X,
    y: OFFSET_Y,
    k: 1,
  });
  const [gridProps] = useState<GridProps>({
    point: { x: GRID_OFFSET_X, y: GRID_OFFSET_Y },
    point2: { x: GRID_WIDTH / 2, y: GRID_HEIGHT / 2 },
    count: GRID_COUNT,
  });

  const clickItemHandler = useCallback(
    (graphNode: GraphNode | undefined) => {
      // let itemType = "node";
      // if(item){
      //   if (item.hasOwnProperty("groupName")) {
      //     itemType = "group";
      //   } else if (item.hasOwnProperty("type")) {
      //     itemType = "link";
      //   }
      // }
      if (onSelected) onSelected(graphNode);
    },
    [onSelected]
  );

  return (
    <SVG width={size.width} height={size.height}>
      <ZoomContainer
        initTransform={zoomTransform}
        onTransform={(transform) => setZoomTransform(transform)}
      >
        {/* <Grid {...gridProps} /> */}
        <GraphLayout
          nodes={graphNodes}
          onLayoutDidUpdate={onLayoutDidUpdate}
          onClickItem={clickItemHandler}
          selectedId={selectedId}
        />
      </ZoomContainer>
    </SVG>
  );
}
