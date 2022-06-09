import { useEffect } from "react";
import * as d3 from "d3";
import { useSvg } from "./SVG";
import {Transform} from "../types";

export type ZoomContainerProps = {
  initTransform: Transform;
  children?: any;
  onTransform: (transform: Transform) => void;
};

export default function ZoomContainer({
  initTransform,
  children,
  onTransform:setTransform,
}: ZoomContainerProps) {
  const svgElement = useSvg();

  useEffect((): any => {
    if (!svgElement) return;

    const selection = d3.select(svgElement);
    const zoom: any = d3.zoom().on("zoom", ({ transform }: any) => {
      setTransform(transform);
    });
    selection.call(
      zoom.transform,
      d3.zoomIdentity
        .translate(initTransform.x, initTransform.y)
        .scale(initTransform.k)
    );
    selection.call(zoom);

    return () => selection.on(".zoom", null);
  }, [svgElement]);

  const { x, y, k } = initTransform;

  return (
    <g transform={`translate(${x}, ${y}) scale(${k})`}>
      {/* <rect x="0" y="0" width={"100%"} height={"100%"} fill="#aaaaaa" /> */}
      {children}
    </g>
  );
}
