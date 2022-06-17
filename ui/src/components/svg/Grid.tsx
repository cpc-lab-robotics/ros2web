import * as d3 from "d3";
import { Point, Size } from "../types";
import styles from "./styles/styles.module.css";

export type GridProps = {
  point: Point;
  point2: Point;
  count: number;
};

function Grid({ point, point2, count }: GridProps) {
  const ticlsCount = count || 1;

  const gridX = d3
    .ticks(point.x, point2.x, ticlsCount)
    .map((x, i) => <line key={i} x1={x} x2={x} y1={point.y} y2={point2.y} />);
  const gridY = d3
    .ticks(point.y, point2.y, ticlsCount)
    .map((y, i) => <line key={i} x1={point.x} x2={point2.x} y1={y} y2={y} />);

  return (
    <g className={styles.line}>
      {gridX}
      {gridY}
    </g>
  );
}

export default Grid;
