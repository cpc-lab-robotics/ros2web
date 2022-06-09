import React from "react";
import * as d3 from "d3";

import * as model from "./models";

export interface LinkProps {
  link: model.Link;
  onClick?: (obj: model.Link) => void;
}

export interface LinkState {
  d: string;
}

class Link extends React.Component<LinkProps, LinkState> {
  private static _links: model.Link[] = [];
  private containerRef = React.createRef<SVGAElement>();

  static setLinks(links: model.Link[]) {
    Link._links = links;
  }

  static getNames = (link: model.Link): string[] => {
    const names: string[] = [];

    for (var i = 0; i < Link._links.length; ++i) {
      let l = Link._links[i];

      if (
        (l.source.name === link.source.name &&
          l.target.name === link.target.name) ||
        (l.source.name === link.target.name &&
          l.target.name === link.source.name)
      )
        names.push(l.name);
    }
    return names;
  };

  private _link: model.Link;

  constructor(props: LinkProps) {
    super(props);

    const { link } = props;

    this._link = link;
    link.component = this;

    this.state = {
      d: "",
    };
  }

  componentDidMount() {
    const selection = d3.select(this.containerRef.current);

    selection.on("click", (event: any) => {
      const { onClick } = this.props;
      if (event.defaultPrevented) return; // dragged
      event.preventDefault();
      if (onClick) onClick(this._link);
    });
    
    // selection.on("dblclick", (event: any) => {
    //   event.stopPropagation();
    // });

  }

  // componentDidUpdate() {}
  // componentWillUnmount() {}

  update() {
    const arc = this.arcPath();
    this.setState({ d: arc });
  }
  render() {
    return (
      <g ref={this.containerRef}>
        <path
          markerEnd="url(#arrow)"
          stroke="#23324F"
          strokeWidth="1"
          fill="none"
          d={this.state.d}
        />
        <path fill="none" id={`label-${this._link.id}`} d={this.state.d} />
        <text>
          <textPath
            startOffset="50%"
            textAnchor="middle"
            xlinkHref={`#label-${this._link.id}`}
            fill="#23324F"
            fontSize="14"
          >
            {this._link.name}
          </textPath>
        </text>
      </g>
    );
  }

  checkLoopBack(source: model.Node, target: model.Node) {
    return source == target;
  }

  // https://bl.ocks.org/mattkohl/146d301c0fc20d89d85880df537de7b0
  arcPath = (leftHand: boolean = true) => {
    const x1 = leftHand ? this._link.source.x : this._link.target.x;

    const y1 = leftHand ? this._link.source.y : this._link.target.y;
    const x2 = leftHand ? this._link.target.x : this._link.source.x;
    const y2 = leftHand ? this._link.target.y : this._link.source.y;

    const dx = x2 - x1;
    const dy = y2 - y1;
    const dr = Math.sqrt(dx * dx + dy * dy);
    const sweep = leftHand ? 0 : 1;
    const xRotation = 0;
    const largeArc = 0;

    if (this.checkLoopBack(this._link.source, this._link.target)) {
      const offset = 70;
      return `M${x1},${y1} C${x1 - offset},${y1},${x1},${
        y1 - offset
      } ${x2},${y2}`;
    }

    const names = Link.getNames(this._link);

    let drx = dr;
    let dry = dr;

    let count = names.length;
    if (count > 1) {
      let arcScale = d3.scaleOrdinal().domain(names).range([1, count]);

      drx = drx / (1 + (1 / count) * (Number(arcScale(this._link.name)) - 1));
      dry = dry / (1 + (1 / count) * (Number(arcScale(this._link.name)) - 1));
    }

    return `M${x1},${y1}A${drx},${dry} ${xRotation},${largeArc},${sweep} ${x2},${y2}`;
  };
}

export default Link;
