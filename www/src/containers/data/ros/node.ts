import React from "react";

import { useProps } from "@/services/api";
import { Node, isNodeKey, NodeKeyType, NodeInterface } from "@/models/node";
import { snakeToCamel, camelToSnake } from "@/utils/replace";

import ROSDataAdaptor, { ROSDataAdaptorProps } from "./base";

import { GraphNode, GraphLink } from "@/components/Graph/types";
import { ItemData } from "@/components/List/types";
import { InfoData } from "@/components/Info/types";

import { Topic } from "@/models/topic";

export type NodeDataAdaptorProps = {
  nodes?: Node[] | Node;
  primary_key?: string;
  secondary_key?: string;
} & ROSDataAdaptorProps;

export default class NodeDataAdaptor extends ROSDataAdaptor {
  private nodes: Node[];
  private primaryKey: NodeKeyType;
  private secondaryKey?: NodeKeyType;
  private topicDict: Record<string, Topic>

  constructor(props: NodeDataAdaptorProps) {
    super(props);
    const { bindStateKey, nodes, primary_key, secondary_key } =
      props;

    this.topicDict = {}
    let primaryKey: NodeKeyType = "name";
    if (primary_key) {
      const key = snakeToCamel(primary_key);
      if (isNodeKey(key)) primaryKey = key;
    }
    this.primaryKey = primaryKey;

    let secondaryKey: NodeKeyType | undefined;
    if (secondary_key) {
      const key = snakeToCamel(secondary_key);
      if (isNodeKey(key)) secondaryKey = key;
    }
    this.secondaryKey = secondaryKey;

    for (const prop of [
      "nodes",
      "secondary_key",
      "secondaryKey",
    ]) {
      const stateKey = bindStateKey[prop];
      if (stateKey) {
        this.bindProps[stateKey] = prop;
      }
    }

    if (nodes) {
      if (Array.isArray(nodes)) {
        this.nodes = nodes;
      } else {
        this.nodes = [nodes];
      }
    } else {
      this.nodes = [];
    }
  }

  get name(): string {
    return this.constructor.name;
  }

  private update() {
    const primary_key = this.updateProps["primary_key"];
    if (primary_key) {
      const primaryKey = snakeToCamel(primary_key);
      if (isNodeKey(primaryKey)) this.primaryKey = primaryKey;
    }
    const secondary_key = this.updateProps["secondary_key"];
    if (secondary_key) {
      const secondaryKey = snakeToCamel(secondary_key);
      if (isNodeKey(secondaryKey)) this.secondaryKey = secondaryKey;
    }
    this.nodes = this.updateProps["nodes"] || this.nodes;
  }

  useItems(): ItemData[] {
    this.update();
    return this.nodes.map((node) => {
      return {
        id: node.id,
        primary: node[this.primaryKey],
        ...(this.secondaryKey ? { secondary: node[this.secondaryKey] } : {}),
      };
    });
  }
  itemToData(item: ItemData): Node | undefined {
    const nodes = this.nodes.filter((node) => node.id === item.id);
    return nodes[nodes.length - 1];
  }

  useGraphData(): GraphNode[] {
    this.update();

    this.topicDict = {};

    const graphNodes: GraphNode[] = this.nodes.map((node) => {
      const outbounds:GraphLink[] = [];
      const inbounds:GraphLink[] = [];

      if (node.interface) {
        if (node.interface.publishers)
          node.interface.publishers.forEach((topic) => {
            this.topicDict[topic.id] = topic;

            const typeName = `${topic.type.packageName}/msg/${topic.type.name}`;
            outbounds.push({
              name: topic.name,
              type: typeName,
              nodeType: "node"
            });
          });

        if (node.interface.subscribers)
          node.interface.subscribers.forEach((topic) => {
            this.topicDict[topic.id] = topic;

            const typeName = `${topic.type.packageName}/msg/${topic.type.name}`;
            inbounds.push({
              name: topic.name,
              type: typeName,
              nodeType: "node"
            });
          });
      }
      return {
        id: node.id,
        name: node[this.primaryKey],
        group: node.namespace === "/" ? undefined : node.namespace,
        inbounds,
        outbounds,
        nodeType: "node",
      };
    });


    for (const [key, topic] of Object.entries(this.topicDict)) {

      const outbounds:GraphLink[] = [];
      const inbounds:GraphLink[] = [];
      // 
      outbounds.push({
        name: topic.name,
        type: `${topic.type.packageName}/msg/${topic.type.name}`,
        nodeType: "topic"
      });
      inbounds.push({
        name: topic.name,
        type: `${topic.type.packageName}/msg/${topic.type.name}`,
        nodeType: "topic"
      });

      graphNodes.push({
        id: topic.id,
        name: topic.name,
        inbounds,
        outbounds,
        nodeType: "topic",
      });
      
    }

    return graphNodes;
  }

  graphDataToData(graphNode:GraphNode): Topic | Node | undefined{
    const nodes = this.nodes.filter((node) => node.id === graphNode.id);
    const node = nodes[nodes.length - 1]

    if(node){
      return node
    }else{
      return this.topicDict[graphNode.id]
    }
    
  }




  // private createLink(
  //   outbound: {
  //     id: string;
  //     topicName: string;
  //     topicType: string;
  //     type: string;
  //   },
  //   inbound: {
  //     id: string;
  //     topicName: string;
  //     topicType: string;
  //     type: string;
  //   }
  // ): GraphLink {
  //   let linkId = `${outbound.id}-${inbound.id}`;

  //   return {
  //     id: linkId,
  //     name: outbound.topicName,
  //     sourceId: outbound.id,
  //     targetId: inbound.id,
  //   };
  // }

  // useGraphData(): [GraphNode[], GraphLink[]] {
  //   this.update();

  //   const outbounds: Array<{
  //     id: string;
  //     topicName: string;
  //     topicType: string;
  //     type: string;
  //   }> = [];

  //   const inbounds: Array<{
  //     id: string;
  //     topicName: string;
  //     topicType: string;
  //     type: string;
  //   }> = [];

  //   const topicDict: Record<string, Topic> = {};

  //   const graphNodes: GraphNode[] = this.nodes.map((node) => {
  //     if (node.interface) {
  //       if (node.interface.publishers)
  //         node.interface.publishers.forEach((topic) => {
  //           outbounds.push({
  //             id: node.id,
  //             topicName: topic.name,
  //             topicType: `${topic.type.packageName}/msg/${topic.type.name}`,
  //             type: "node",
  //           });
  //           topicDict[topic.id] = topic;
  //         });

  //       if (node.interface.subscribers)
  //         node.interface.subscribers.forEach((topic) => {
  //           inbounds.push({
  //             id: node.id,
  //             topicName: topic.name,
  //             topicType: `${topic.type.packageName}/msg/${topic.type.name}`,
  //             type: "node",
  //           });
  //           topicDict[topic.id] = topic;
  //         });
  //     }

  //     return {
  //       id: node.id,
  //       type: "node",
  //       name: node[this.primaryKey],
  //       group: node.namespace === "/" ? undefined : node.namespace,
  //     };
  //   });

  //   for (const [key, topic] of Object.entries(topicDict)) {
  //     graphNodes.push({
  //       id: topic.id,
  //       type: "topic",
  //       name: topic.type.name,
  //     });
  //     outbounds.push({
  //       id: topic.id,
  //       topicName: topic.name,
  //       topicType: `${topic.type.packageName}/msg/${topic.type.name}`,
  //       type: "topic",
  //     });
  //     inbounds.push({
  //       id: topic.id,
  //       topicName: topic.name,
  //       topicType: `${topic.type.packageName}/msg/${topic.type.name}`,
  //       type: "topic",
  //     });
  //   }

  //   const graphLinks: GraphLink[] = [];

  //   for (const outbound of outbounds) {
  //     const filtered = inbounds.filter(
  //       (inbound) =>
  //         outbound.topicName === inbound.topicName &&
  //         outbound.topicType === inbound.topicType
  //         // && outbound.type !== inbound.type
  //     );

  //     for (const inbound of filtered) {
  //       const link = this.createLink(outbound, inbound);
  //       graphLinks.push(link);
  //     }

  //   }

  //   return [graphNodes, graphLinks];
  // }

  // graphNodeToData(graphNode: GraphNode): Node | undefined {
  //   const nodes = this.nodes.filter((node) => node.id === graphNode.id);
  //   return nodes[nodes.length - 1];
  // }

  useInfo(): InfoData | undefined {
    return;
  }
}
