import React from "react";
import * as d3 from "d3";
import { styled } from "@mui/material/styles";

import { Point } from "../../types";
import * as model from "./models";
import { grahamScan as convexHull } from "./convex-hull";

const Text = styled("text")(({ theme }) => ({
  x: 5,
  y: 15,
  fill: "#73819C",
  stroke: "none",
  fontFamily: "sans-serif",
  fontStyle: "italic",
  fontSize: 15,
}));

export interface GroupProps {
  group: model.Group;

  onDragStart?: (obj: model.Group | model.Node) => void;
  onDragging?: (
    obj: model.Group | model.Node,
    position: { x: number; y: number }
  ) => void;
  onDragEnd?: (obj: model.Group | model.Node) => void;
  onClick?: (obj: model.Group) => void;
}

export interface GroupState {
  points: string[];
  fill: string;
  stroke: string;
  strokeWidth: number;
}

export default class GroupComponent extends React.Component<
  GroupProps,
  GroupState
> {
  private containerRef = React.createRef<SVGAElement>();

  private _group: model.Group;
  private _event: any;

  constructor(props: GroupProps) {
    super(props);

    const { group } = props;
    this._group = group;

    // group.padding = 30;
    group.component = this;
    // let x = 0,
    //   y = 0,
    //   width = 50,
    //   height = 50;
    // if (group.bounds) {
    //   x = group.bounds.x;
    //   y = group.bounds.y;
    //   width = group.bounds.width();
    //   height = group.bounds.height();
    // }
    
    this.state = {
      points: [],
      fill: "#DDDDDD",
      stroke: "#DDDDDD",
      strokeWidth: 55,
    };
  }

  componentDidMount() {
    const selection = d3.select(this.containerRef.current);

    const drag: any = d3
      .drag()
      .on("start", (event) => {
        if (this.props.onDragStart) this.props.onDragStart(this._group);
      })
      .on("drag", (event) => {
        this._event = event;
        if (this.props.onDragging) this.props.onDragging(this._group, event);
      })
      .on("end", (event) => {
        if (this.props.onDragEnd) this.props.onDragEnd(this._group);
      });

    selection.call(drag);

    selection.on("click", (event: any) => {
      const { onClick } = this.props;
      if (event.defaultPrevented) return; // dragged
      if (onClick) onClick(this._group);
    });
    
    // selection.on("dblclick", (event: any) => {
    //   event.stopPropagation();
    // });
  }

  // componentDidUpdate() {}
  // componentWillUnmount() {}

  update() {
    if (this._group.groups) {
      let ps: number[][] = [];
      this._group.groups.forEach((sub) => {
        const leaves = sub.leaves || [];
        let _ps = leaves.map((n) => [n.x, n.y]);
        ps = ps.concat(_ps);
      });
      ps = convexHull(ps);
      const points = ps.map((p) => `${p[0]},${p[1]}`);
      this.setState({ points });
    } else {

      if(this._group.name !== "_"){
        const leaves = this._group.leaves || [];
        let ps = leaves.map((n) => [n.x, n.y]);
        ps = convexHull(ps);
        const points = ps.map((p) => `${p[0]},${p[1]}`);
        this.setState({ points, fill:"#aaaaaa", stroke: "#aaaaaa", strokeWidth: 50});
      }else{
        this.setState({ points:[] });
      }
    }
  }

  render() {
    const { points, fill, stroke, strokeWidth } = this.state;
    return (
      <g ref={this.containerRef}>
        <polygon
          points={points.join(" ")}
          fill={fill}
          strokeWidth={strokeWidth}
          strokeLinejoin={"round"}
          strokeLinecap={"round"}
          stroke={stroke}
        />
      </g>
    );
  }
}
