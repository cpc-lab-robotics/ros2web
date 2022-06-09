import React, { Dispatch } from "react";
import { getStateValue } from "@/services/api";

import { GraphNode, GraphLink } from "@/components/Graph/types";
import { ItemData } from "@/components/List/types";
import { InfoData } from "@/components/Info/types";

import { Data } from "@/models";

export type ROSDataAdaptorProps = {
  webPackageName: string;
  bindStateKey: Record<string, string>;
  initState: Record<string, any>;
  setState: Dispatch<Record<string, any>>;
};

export default abstract class ROSDataAdaptor {
  protected webPackageName: string;
  protected bindProps: Record<string, string>;
  protected initState: Record<string, any>;
  protected updateProps: Record<string, any>;

  constructor(props: ROSDataAdaptorProps) {
    const { webPackageName, initState } = props;
    this.webPackageName = webPackageName;
    this.initState = initState;
    this.bindProps = {};
    this.updateProps = {};
  }

  get queries() {
    return Object.keys(this.bindProps).map((key: string) => ({
      queryKey: [this.webPackageName, "state", key],
      queryFn: () => getStateValue(this.webPackageName, key),
      initialData: this.initState[key],
    }));
  }

  set_results(results: any[]) {
    Object.keys(this.bindProps).map((key: string, index) => {
      const { data } = results[index];
      const propName = this.bindProps[key];
      this.updateProps[propName] = data;
    });
  }

  abstract get name(): string;
  abstract useItems(): ItemData[];
  abstract itemToData(item: ItemData): Data | undefined;

  abstract useGraphData(): GraphNode[];
  abstract graphDataToData(graphNode:GraphNode): Data | undefined;
  // abstract graphNodeToData(graphNode: GraphNode): Data | undefined;
  // abstract getSelectedGraphNode(): GraphNode | undefined;

  abstract useInfo(): InfoData | undefined
}
