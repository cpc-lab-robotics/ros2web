import React from "react";

import * as d3 from "d3";

export function MarkerDef() {
  // const markerEl = React.useRef(null);

  const radius = 15;
  const markerBoxWidth = 10;
  const markerBoxHeight = 10;

  const refX = markerBoxWidth + radius;
  const refY = markerBoxHeight* 0.6;

  const pathData = `M0,0L0,${markerBoxHeight}L${markerBoxHeight},${markerBoxHeight / 2}`;
  
  return (
    <marker
      id="arrow"
      viewBox={[0, 0, markerBoxWidth, markerBoxHeight].toString()}
      refX={refX}
      refY={refY}
      markerWidth={markerBoxWidth}
      markerHeight={markerBoxHeight}
      orient="auto"
      markerUnits="strokeWidth"
      overflow="visible"
    >
      <path d={pathData} fill="#23324F"></path>
    </marker>
  );
}
