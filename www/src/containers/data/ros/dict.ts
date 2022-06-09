import React from "react";

import { useProps } from "@/services/api";
import { Node, isNodeKey, NodeKeyType, NodeInterface } from "@/models/node";
import { snakeToCamel, camelToSnake } from "@/utils/replace";

import ROSDataAdaptor, { ROSDataAdaptorProps } from "./base";

import { GraphNode, GraphLink } from "@/components/Graph/types";
import { ItemData } from "@/components/List/types";
import { InfoData, KeyValueWithHeader, KeyValue } from "@/components/Info/types";

import { Topic } from "@/models/topic";

export type DictDataAdaptorProps = {
  dict?: Array<Record<string, any>> | Record<string, any>;
  primary_key?: string;
  secondary_key?: string;
} & ROSDataAdaptorProps;

export default class NodeDataAdaptor extends ROSDataAdaptor {
  private dict: Array<Record<string, any>> | Record<string, any>;
  private primaryKey?: string;
  private secondaryKey?: string;
  
  constructor(props: DictDataAdaptorProps) {
    super(props);
    const { bindStateKey, dict, primary_key, secondary_key } = props;

    this.dict = dict || {};

    this.primaryKey = primary_key;
    this.secondaryKey = secondary_key;

    for (const prop of ["dict", "secondary_key", "secondaryKey"]) {
      const stateKey = bindStateKey[prop];
      if (stateKey) {
        this.bindProps[stateKey] = prop;
      }
    }
  }

  get name(): string {
    return this.constructor.name;
  }

  private update() {
    const primary_key = this.updateProps["primary_key"];
    if (primary_key) {
      this.primaryKey = primary_key;
    }
    const secondary_key = this.updateProps["secondary_key"];
    if (secondary_key) {
      this.secondaryKey = secondary_key;
    }

    this.dict = this.updateProps["dict"] || this.dict;
  }

  useItems(): ItemData[] {
    this.update();

    // return this.nodes.map((node) => {
    //   return {
    //     id: node.id,
    //     primary: node[this.primaryKey],
    //     ...(this.secondaryKey ? { secondary: node[this.secondaryKey] } : {}),
    //   };
    // });

    return [];
  }
  itemToData(item: ItemData): Node | undefined {
    // const nodes = this.nodes.filter((node) => node.id === item.id);
    // return nodes[nodes.length - 1];
    return;
  }

  useGraphData(): GraphNode[] {
    return [];
  }

  graphDataToData(graphNode: GraphNode): Topic | Node | undefined {
    return;
  }

  useInfo(): InfoData | undefined {
    this.update();

    const items: Array<KeyValueWithHeader | KeyValue> = [];
    for (const [key, value] of Object.entries(this.dict)) {
      const header = undefined;
      if (typeof value === "object") {
        const header = key;
        const keyValues: Array<KeyValue> =  []
        for (const [key2, value2] of Object.entries(value)) {
          keyValues.push({
            key: key2,
            value: value2,
          })
        }
        items.push({
          header,
          keyValues,
        });
      } else {
        const keyValue: KeyValue = {
          key,
          value,
        };
        items.push(keyValue);
      }
    }

    return {
      title: "",
      items,
    };
  }
}
