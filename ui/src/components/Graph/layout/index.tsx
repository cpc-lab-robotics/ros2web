import React from "react";
import { GraphNode, GraphGroup, GraphLink } from "../types";
import Layout from "./layout-state";
import { MarkerDef } from "./defs";

import * as model from "./models";

import Group from "./group";
import Node from "./node";
import Link from "./link";

type GraphLayoutProps = {
  nodes?: GraphNode[];
  onClickItem?: (item: GraphNode) => void;
  onLayoutDidUpdate?: () => void;
  selectedId?: string;
};

type GraphLayoutState = {};

export default class GraphLayout extends React.Component<
  GraphLayoutProps,
  GraphLayoutState
> {
  private layout: Layout;

  constructor(props: GraphLayoutProps) {
    super(props);
    const { nodes: _nodes, selectedId } = props;
    const nodes = _nodes || [];
    this.layout = new Layout({
      onLayoutDidStart: this.onLayoutDidStart,
      onLayoutDidUpdate: this.onLayoutDidUpdate,
      onLayoutDidStop: this.onLayoutDidStop,
    });
    this.layout.setData(nodes);

    if (selectedId) {
      // let itemId:string = "";
      // let itemType:"node" | "group" | "link" = "node";
      // if (selectedItem.hasOwnProperty("groupName")) {
      //   const group = selectedItem as GraphGroup;
      //   itemId = group.groupName;
      //   itemType = "group";
      // } else if (selectedItem.hasOwnProperty("type")) {
      //   itemType = "link";

      // }else{
      //   const node = selectedItem as GraphNode;
      //   itemId = node.name;
      // }
      
      this.layout.selectItem(selectedId, "node");
    }

    this.layout.update({
      initialUnconstrainedIterations: 0,
      initialUserConstraintIterations: 0,
      initialAllConstraintsIterations: 0,
      gridSnapIterations: 0,
      keepRunning: true,
      centerGraph: false,
    });
  }

  shouldComponentUpdate(
    nextProps: GraphLayoutProps,
    nextState: GraphLayoutState
  ) {
    const { nodes, selectedId } = nextProps;

    let resume = false;
    if (selectedId != this.props.selectedId) {
      if (selectedId) {
        // if (selectedItem.hasOwnProperty("groupName")) {
        // } else if (selectedItem.hasOwnProperty("type")) {
        // } else {
        //   const node = selectedItem as GraphNode;
        //   this.layout.selectItem(node.name, "node");
        //   resume = true;
        // }
        this.layout.selectItem(selectedId, "node");
        resume = true;
      } else {
        this.layout.selectItem(null);
        resume = true;
      }
    }

    if (nodes && this.props.nodes !== nodes) {
      this.layout.setData(nodes);
      this.layout.update({
        initialUnconstrainedIterations: 0,
        initialUserConstraintIterations: 0,
        initialAllConstraintsIterations: 0,
        gridSnapIterations: 0,
        keepRunning: true,
        centerGraph: false,
      });
      return true;
    } else {
      if (resume) this.layout.resume();

      return false;
    }
  }

  componentDidUpdate(prevProps: GraphLayoutProps, prevState: GraphLayoutState) {
    // console.log("componentDidUpdate");
  }
  onLayoutDidStart = () => {};
  onLayoutDidUpdate = () => {
    let groups = (this.layout.groups as model.Group[]) || [];
    groups.forEach((g) => {
      if (g.component) g.component.update();
    });

    let subGroups = (this.layout.subGroups as model.Group[]) || [];
    subGroups.forEach((g) => {
      if (g.component) g.component.update();
    });

    let nodes = (this.layout.nodes as model.Node[]) || [];
    nodes.forEach((n) => {
      if (n.component) n.component.update();
    });

    let links = (this.layout.links as model.Link[]) || [];
    links.forEach((l) => {
      if (l.component) l.component.update();
    });
  };
  onLayoutDidStop = () => {
    const { onLayoutDidUpdate } = this.props;

    if (onLayoutDidUpdate) onLayoutDidUpdate();
  };

  onDragStart = (obj: model.Group | model.Node) => {
    Layout.onDragStart(obj);
  };
  onDragging = (
    obj: model.Group | model.Node,
    position: { x: number; y: number }
  ) => {
    Layout.onDragging(obj, position);
    this.layout.resume();
  };
  onDragEnd = (obj: model.Group | model.Node) => {
    Layout.onDragEnd(obj);
  };
  onClickNode = (node: model.Node) => {
    const { onClickItem } = this.props;
    const gn: GraphNode = {
      id: node.id,
      name: node.name,
      nodeType: node.nodeType,
    };
    
    this.layout.selectItem(node.name, "node");

    if (onClickItem) onClickItem(gn);
  };

  onClickGroup = (group: model.Group) => {
    // const { onClickItem } = this.props;

    // let groupName;
    // let subGroupName;

    // let gropType: "group" | "subgroup";
    // if (group.parent) {
    //   groupName = group.parent.name;
    //   subGroupName = group.name;
    //   gropType = "group";
    // } else {
    //   groupName = group.name;
    //   gropType = "subgroup";
    // }

    // const gg: GraphGroup = {
    //   groupName,
    //   ...(subGroupName ? { subGroupName } : {}),
    // };

    // this.layout.selectItem(group.id, gropType);

    // if (onClickItem) onClickItem(gg);
  };

  onClickLink = (link: model.Link) => {
    // const { onClickItem } = this.props;

    // const gl: GraphLink = {
    //   name: link.name,
    //   type: link.type,
    // };

    // this.layout.selectItem(link.id, "link");

    // if (onClickItem) onClickItem(gl);
  };

  render() {
    Link.setLinks(this.layout.links);

    return (
      <g>
        <defs>
          <MarkerDef />
        </defs>

        <g>
          {this.layout.groups.map((group) => (
            <Group
              key={group.id}
              group={group}
              onDragStart={this.onDragStart}
              onDragging={this.onDragging}
              onDragEnd={this.onDragEnd}
              onClick={this.onClickGroup}
            />
          ))}
        </g>

        <g>
          {this.layout.subGroups.map((group) => (
            <Group
              key={group.id}
              group={group}
              onDragStart={this.onDragStart}
              onDragging={this.onDragging}
              onDragEnd={this.onDragEnd}
              onClick={this.onClickGroup}
            />
          ))}
        </g>

        <g>
          {this.layout.links.map((link) => (
            <Link key={link.id} link={link} onClick={this.onClickLink} />
          ))}
        </g>

        <g>
          {this.layout.nodes.map((node) => (
            <Node
              key={node.id}
              node={node}
              onDragStart={this.onDragStart}
              onDragging={this.onDragging}
              onDragEnd={this.onDragEnd}
              onClick={this.onClickNode}
            />
          ))}
        </g>
      </g>
    );
  }
}
