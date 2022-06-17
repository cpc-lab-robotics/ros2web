import React from "react";
import * as d3 from "d3";
import { styled } from "@mui/material/styles";
import * as model from "./models";

import { Size, Point } from "../../types";

interface CircleProps {
  selected?: boolean;
}

const Circle = styled("circle", {
  shouldForwardProp: (prop) => prop !== "selected",
})<CircleProps>(({ selected, theme }) => ({
  alpha: 1,
  strokeWidth: 5,
  strokeDasharray: "none",
  stroke: theme.palette.primary.main,
  cursor: "auto",
  fill: "white",
  ...(selected && {
    fill: theme.palette.primary.main,
  }),
}));

interface RectProps {
  selected?: boolean;
}

const Rect = styled("rect", {
  shouldForwardProp: (prop) => prop !== "selected",
})<RectProps>(({ selected, theme }) => ({
  alpha: 1,
  strokeWidth: 5,
  strokeDasharray: "none",
  stroke: theme.palette.primary.main,
  cursor: "auto",
  fill: "white",
  ...(selected && {
    fill: theme.palette.primary.main,
  }),
}));

const Text = styled("text")(({ theme }) => ({
  fontFamily: "sans-serif",
  stroke: "none",
  fill: theme.palette.secondary.main,
  fontWeight: "bold",
  fontSize: 15,
  cursor: "default",
  userSelect: "none",
}));

export interface NodeProps {
  node: model.Node;

  onDragStart?: (obj: model.Group | model.Node) => void;
  onDragging?: (obj: model.Group | model.Node, position: Point) => void;
  onDragEnd?: (obj: model.Group | model.Node) => void;
  onClick?: (obj: model.Node) => void;
}

export interface NodeState {
  name: string;
  point: Point;
  radius: number;
  selected: boolean;
  active: boolean;
  size: Size;
  nodeType: string;
}

class Node extends React.Component<NodeProps, NodeState> {
  private containerRef = React.createRef<SVGAElement>();
  private _node: model.Node;
  private _event: any;

  constructor(props: NodeProps) {
    super(props);

    const { node } = props;

    this._node = node;
    node.component = this;

    this.state = {
      nodeType: node.nodeType,
      name: node.name,
      point: { x: node.x, y: node.y },
      radius: 15,
      selected: node.selected,
      active: node.active,
      size: { width: node.width || 30, height: node.height || 30 },
    };
  }

  componentDidMount() {
    const selection = d3.select(this.containerRef.current);

    const drag: any = d3
      .drag()
      .on("start", (event) => {
        if (this.props.onDragStart) this.props.onDragStart(this._node);
      })
      .on("drag", (event) => {
        this._event = event;

        if (this.props.onDragging) this.props.onDragging(this._node, event);
      })
      .on("end", (event) => {
        if (this.props.onDragEnd) this.props.onDragEnd(this._node);
      });

    selection.call(drag);
    selection.on("click", (event: any) => {
      const { onClick } = this.props;
      if (event.defaultPrevented) return; // dragged
      event.preventDefault();
      if (onClick) onClick(this._node);
    });

    selection.on("dblclick", (event: any) => {
      event.stopPropagation();
    });
  }

  componentDidUpdate() {}
  componentWillUnmount() {}

  update() {
    this.setState({
      point: { x: this._node.x, y: this._node.y },
      selected: this._node.selected,
      active: this._node.active,
    });
  }

  render() {
    const { point, radius, name, selected, active, size, nodeType } = this.state;
    const w = size.width * 0.7;

    return (
      <g
        ref={this.containerRef}
        transform={`translate(${point.x}, ${point.y})`}
        // className={styles.node}
        width={size.width}
        height={size.height}
        opacity={active ? 1.0 : 0.2}
      >
        {nodeType === "node" ? (
          <Circle r={radius} selected={selected} />
        ) : (
          <Rect x={-w/2} y={-w/2} width={w} height={w} selected={selected} transform="rotate(45)" />
        )}

        <Text x={20} y={".31em"} focusable={false}>
          {name}
        </Text>
      </g>
    );
  }
}
export default Node;
