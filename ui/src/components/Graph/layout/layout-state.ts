import * as webcola from "webcola";
import * as d3 from "d3";

import { GraphNode, GraphLink } from "../types";
import { Node, Group, Link } from "./models";

class WebcolaLayout extends webcola.Layout {
  kick() {
    // var t = d3.timer(() => super.tick() && t.stop());

    let timer = d3.timer((elapsed) => {
      if (elapsed > 2000) {
        super.stop();
      }
      super.tick() && timer.stop();
    });
  }
}

function isGroup(g: any): g is Group {
  return typeof g.leaves !== "undefined" || typeof g.groups !== "undefined";
}

export type LayoutProps = {
  onLayoutDidStart?: () => void;
  onLayoutDidUpdate?: () => void;
  onLayoutDidStop?: () => void;
};

type LayoutConfig = {
  node: {
    initialPosition: { x: number; y: number };
    size: { width: number; height: number };
  };
  group: {
    padding: number;
  };
  subGroup: {
    padding: number;
  };
  layout: {
    initialUnconstrainedIterations: number;
    initialUserConstraintIterations: number;
    initialAllConstraintsIterations: number;
    gridSnapIterations: number;
    keepRunning: boolean;
    centerGraph: boolean;
  };
};

const LAYOUT_CONFIG: LayoutConfig = {
  node: {
    initialPosition: { x: 0, y: 0 },
    size: { width: 30, height: 30 },
  },
  group: {
    padding: 1,
  },
  subGroup: {
    padding: 1,
  },
  layout: {
    initialUnconstrainedIterations: 0,
    initialUserConstraintIterations: 0,
    initialAllConstraintsIterations: 0,
    gridSnapIterations: 0,
    keepRunning: false,
    centerGraph: false,
  },
};
export default class LayoutState {
  private _layout: WebcolaLayout;

  private _nodes: Node[];
  private _subGroups: Group[];
  private _groups: Group[];
  private _links: Link[];

  private _nodeDict: { [id: string]: Node };
  private _subGroupDict: { [id: string]: Group };
  private _groupDict: { [id: string]: Group };
  private _linkDict: { [id: string]: Link };

  private _updateable: boolean;
  private _config: LayoutConfig;

  private _selectedId: { [id: string]: string | null };

  constructor(props: LayoutProps) {
    const { onLayoutDidStart, onLayoutDidUpdate, onLayoutDidStop } = props;

    this._layout = new WebcolaLayout();

    if (onLayoutDidStart) {
      this._layout.on("start", onLayoutDidStart);
    }
    if (onLayoutDidUpdate) {
      this._layout.on("tick", onLayoutDidUpdate);
    }
    if (onLayoutDidStop) {
      this._layout.on("end", onLayoutDidStop);
    }

    this._layout.linkDistance(150);
    this._layout.avoidOverlaps(true);
    this._layout.handleDisconnected(false);

    this._nodes = [];
    this._subGroups = [];
    this._groups = [];
    this._links = [];

    this._nodeDict = {};
    this._subGroupDict = {};
    this._groupDict = {};
    this._linkDict = {};

    this._updateable = false;

    this._selectedId = {};

    this._config = { ...LAYOUT_CONFIG };
  }

  get nodes(): Node[] {
    return this._nodes;
  }

  get subGroups(): Group[] {
    return this._subGroups;
  }

  get groups(): Group[] {
    return this._groups;
  }

  get links(): Link[] {
    return this._links;
  }

  public setConfig(config: { [id: string]: any }) {
    this._config = { ...LAYOUT_CONFIG, ...config };
  }
  public resume() {
    if (this._nodes.length > 0) {
      this._layout.resume();
    }
  }

  public stop() {
    this._layout.stop();
  }

  selectItem(
    itemId: string | null,
    itemType?: "node" | "group" | "subgroup" | "link"
  ) {
    const selectedId: { [id: string]: string | null } = {
      node: null,
      group: null,
      subgroup: null,
      link: null,
    };

    if (itemType) selectedId[itemType] = itemId;

    this._selectedId = selectedId;

    this._nodes.forEach((n: Node) => {
      n.selected = n.id === selectedId["node"];
    });
    this._groups.forEach((g: Group) => {
      g.selected = g.id === selectedId["group"];
    });
    this._subGroups.forEach((sg: Group) => {
      sg.selected = sg.id === selectedId["subgroup"];
    });
    this._links.forEach((l: Link) => {
      l.selected = l.name === selectedId["link"];
    });
  }

  public update = (props: {
    initialUnconstrainedIterations?: number;
    initialUserConstraintIterations?: number;
    initialAllConstraintsIterations?: number;
    gridSnapIterations?: number;
    keepRunning?: boolean;
    centerGraph?: boolean;
  }) => {
    if (this._updateable) {
      const {
        initialUnconstrainedIterations,
        initialUserConstraintIterations,
        initialAllConstraintsIterations,
        gridSnapIterations,
        keepRunning,
        centerGraph,
      } = props;

      const _initialUnconstrainedIterations = initialUnconstrainedIterations
        ? initialUnconstrainedIterations
        : this._config.layout.initialUnconstrainedIterations;
      const _initialUserConstraintIterations = initialUserConstraintIterations
        ? initialUserConstraintIterations
        : this._config.layout.initialUserConstraintIterations;
      const _initialAllConstraintsIterations = initialAllConstraintsIterations
        ? initialAllConstraintsIterations
        : this._config.layout.initialAllConstraintsIterations;
      const _gridSnapIterations = gridSnapIterations
        ? gridSnapIterations
        : this._config.layout.gridSnapIterations;
      const _keepRunning =
        keepRunning !== undefined
          ? keepRunning
          : this._config.layout.keepRunning;
      const _centerGraph =
        centerGraph !== undefined
          ? centerGraph
          : this._config.layout.centerGraph;

      const _groups: Group[] = [...this._subGroups, ...this._groups];
      if (this._nodes.length > 0) {
        this._layout
          .nodes(this._nodes)
          .links(this._links)
          .groups(_groups)
          .jaccardLinkLengths(150, 0.7)
          .avoidOverlaps(true)
          .start(
            _initialUnconstrainedIterations,
            _initialUserConstraintIterations,
            _initialAllConstraintsIterations,
            _gridSnapIterations,
            _keepRunning,
            _centerGraph
          );
      }
      this._updateable = false;
    }
  };

  private createNode(graphNode: GraphNode): Node {
    const { id, name, subGroup, group, x, y, pin, active, nodeType } = graphNode;

    let node: Node;
    const nodeId = id;

    if (nodeId in this._nodeDict) {
      node = this._nodeDict[nodeId];
      node.active = active === undefined ? true : active;
      node.selected = this._selectedId["node"] === nodeId;
    } else {
      node = {
        id: nodeId,
        name,
        subGroup,
        group,
        x: x !== undefined ? x : this._config.node.initialPosition.x,
        y: y !== undefined ? y : this._config.node.initialPosition.y,
        pin: !!pin,
        active: active === undefined ? true : active,
        selected: this._selectedId["node"] === nodeId,
        nodeType,
        width: this._config.node.size.width,
        height: this._config.node.size.height,
      };
    }
    return node;
  }

  private createLink(
    outbound: { node: Node; topic: GraphLink },
    inbound: { node: Node; topic: GraphLink }
  ): Link {
    let linkId = `${outbound.node.id}-${outbound.topic.name}-${outbound.topic.type}-${inbound.node.id}`;

    let link: Link;

    if (linkId in this._linkDict) {
      link = this._linkDict[linkId];
      link.name = outbound.topic.name;
      link.source = outbound.node;
      link.target = inbound.node;
      link.selected = this._selectedId["link"] === linkId;
    } else {
      link = {
        id: linkId,
        name: outbound.topic.name,
        type: outbound.topic.type,
        source: outbound.node,
        target: inbound.node,
        selected: this._selectedId["link"] === linkId,
      };
    }
    return link;
  }

  private createGroup(
    node: Node,
    newGroupDict: { [id: string]: Group }
  ): Group | undefined {
    let group: Group | undefined;

    if (node.group) {
      if (node.group in newGroupDict) {
        group = newGroupDict[node.group];
      } else if (node.group in this._groupDict) {
        group = this._groupDict[node.group];
        group.selected = this._selectedId["group"] === node.group;
        group.groups = [];
        newGroupDict[node.group] = group;
      } else {
        group = {
          id: node.group,
          name: node.group,
          groups: [],
          padding: this._config.group.padding,
          selected: this._selectedId["group"] === node.group,
        };
        newGroupDict[node.group] = group;
      }
    }
    return group;
  }

  private createSubGroup(
    node: Node,
    newSubGroupDict: { [id: string]: Group }
  ): Group | undefined {
    let subGroup: Group | undefined;
    if (node.group) {
      const subGroupName = node.subGroup ? node.subGroup : "_";
      const subGroupId = `${node.group}-${subGroupName}`;
      if (subGroupId in newSubGroupDict) {
        subGroup = newSubGroupDict[subGroupId];
      } else if (subGroupId in this._subGroupDict) {
        subGroup = this._subGroupDict[subGroupId];
        subGroup.selected = this._selectedId["subgroup"] === subGroupId;
        subGroup.leaves = [];
        newSubGroupDict[subGroupId] = subGroup;
      } else {
        subGroup = {
          id: subGroupId,
          name: subGroupName,
          leaves: [],
          padding: this._config.subGroup.padding,
          selected: this._selectedId["subgroup"] === subGroupId,
        };
        newSubGroupDict[subGroupId] = subGroup;
      }
    }
    return subGroup;
  }

  public setData(graphNodes: GraphNode[]) {
    const newNodeDict: { [id: string]: Node } = {};

    const newGroupDict: { [id: string]: Group } = {};
    const newSubGroupDict: { [id: string]: Group } = {};

    const newLinkDict: { [id: string]: Link } = {};
    const outbounds: Array<{ node: Node; topic: GraphLink }> = [];
    const inbounds: Array<{ node: Node; topic: GraphLink }> = [];

    graphNodes.forEach((gn) => {
      let node = this.createNode(gn);
      newNodeDict[node.id] = node;

      let group: Group | undefined = this.createGroup(node, newGroupDict);
      let subGroup: Group | undefined = this.createSubGroup(
        node,
        newSubGroupDict
      );

      // console.log(group, newGroupDict);

      if (group && subGroup) {
        if (!subGroup.leaves?.includes(node)) {
          subGroup.leaves?.push(node);
          node.parent = subGroup;
        }

        if (!group.groups?.includes(subGroup)) {
          group.groups?.push(subGroup);
          subGroup.parent = group;
        }
      }

      if (gn.outbounds)
        gn.outbounds.forEach((topic) => outbounds.push({ node, topic }));
      if (gn.inbounds)
        gn.inbounds.forEach((topic) => inbounds.push({ node, topic }));
    });

    for (const outbound of outbounds) {
      const filtered = inbounds.filter(
        (inbound) =>
          outbound.topic.name === inbound.topic.name &&
          outbound.topic.type === inbound.topic.type &&
          outbound.topic.nodeType !== inbound.topic.nodeType
      );

      for (const inbound of filtered) {
        const link = this.createLink(outbound, inbound);
        newLinkDict[link.id] = link;
      }
    }

    this._nodeDict = newNodeDict;
    this._groupDict = newGroupDict;
    this._subGroupDict = newSubGroupDict;
    this._linkDict = newLinkDict;

    this._nodes = Object.values(this._nodeDict);
    this._groups = Object.values(this._groupDict);
    this._subGroups = Object.values(this._subGroupDict);
    this._links = Object.values(this._linkDict);

    this._updateable = true;
  }

  static onDragStart(obj: Group | Node) {
    if (!isGroup(obj)) {
      obj.pin = true;
    }
    WebcolaLayout.dragStart(obj);
  }
  static onDragging(obj: Group | Node, position: { x: number; y: number }) {
    WebcolaLayout.drag(obj, position);
  }

  static onDragEnd(obj: Group | Node) {
    WebcolaLayout.dragEnd(obj);

    if (isGroup(obj)) {
      const group = obj;
      const leaves = group.leaves as Node[];
      if (leaves) {
        leaves.forEach((n: Node) => {
          if (n.pin && n.fixed !== undefined) {
            n.fixed |= 6;
          }
        });
      }
    } else {
      if (obj.pin && obj.fixed !== undefined) {
        obj.fixed |= 6;
      }
    }
  }
}
