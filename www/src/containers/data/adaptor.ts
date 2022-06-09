import React, { Dispatch } from "react";
import { useQueries } from "react-query";

import { Node, isNodeKey, NodeKeyType, NodeInterface } from "@/models/node";
import { snakeToCamel, camelToSnake } from "@/utils/replace";

import { GraphNode, GraphLink } from "@/components/Graph/types";
import { ItemData } from "@/components/List/types";
import { InfoData } from "@/components/Info/types";

import { Topic } from "@/models/topic";
import { Data } from "@/models";

import ROSDataAdaptor from "./ros/base";

export type DataAdaptorProps = {
  rosDataAdaptors: ROSDataAdaptor[];
};

export default class DataAdaptor {
  protected rosDataAdaptors: ROSDataAdaptor[];
  private sliceDict: { [id: string]: any };
  private queries: any[];

  constructor(props: DataAdaptorProps) {
    const { rosDataAdaptors } = props;

    this.rosDataAdaptors = rosDataAdaptors;
    this.sliceDict = {};

    let queries: any[] = [];
    for (const adaptor of this.rosDataAdaptors) {
      const start = queries.length;
      const end = queries.length + adaptor.queries.length;
      this.sliceDict[adaptor.name] = [start, end];
      queries = queries.concat(adaptor.queries);
    }
    this.queries = queries;
  }

  private update(){
    const results = useQueries(this.queries);
    for (const adaptor of this.rosDataAdaptors) {
      const [start, end] = this.sliceDict[adaptor.name];
      adaptor.set_results(results.slice(start, end));
    }
  }
  useItems(): ItemData[] {
    this.update()

    let data: ItemData[] = [];
    for (const adaptor of this.rosDataAdaptors) {
      const items = adaptor.useItems();
      data = data.concat(items);
      break;
    }

    return data;
  }
  itemToData(item: ItemData): Data | undefined {
    for (const adaptor of this.rosDataAdaptors) {
      const data = adaptor.itemToData(item);
      if (data) {
        return data;
      }
    }
    return;
  }

  useGraphData(): GraphNode[] {
    this.update()

    let graphNodes: GraphNode[] = [];
    for (const adaptor of this.rosDataAdaptors) {
      graphNodes = adaptor.useGraphData();
      break;
    }
    return graphNodes
  }

  graphDataToData(graphNode: GraphNode): Data | undefined {
    for (const adaptor of this.rosDataAdaptors) {
      const data = adaptor.graphDataToData(graphNode);
      if (data) {
        return data;
      }
    }
    return;
  }


  // useGraphData(): [GraphNode[], GraphLink[]] {
  //   this.update()

  //   let graphNodes: GraphNode[] = [];
  //   let graphLinks: GraphLink[] = [];

  //   for (const adaptor of this.rosDataAdaptors) {
  //     const [gns, gls] = adaptor.useGraphData();
  //     graphNodes = gns;
  //     graphLinks = gls;
  //     break;
  //   }
    
  //   return [graphNodes, graphLinks];
  // }
  // graphNodeToData(graphNode: GraphNode): Data | undefined{

  // }
  // getSelectedGraphNode(): GraphNode | undefined{

  // }

  useInfo(): InfoData | undefined {
    this.update()

    for (const adaptor of this.rosDataAdaptors) {
      const info = adaptor.useInfo();
      return info
    }
    
    return;
  }
}
