import React, { Dispatch } from "react";
import { useQueries } from "react-query";

import { getStateValue } from "@/services/api";
import { Data } from "@/models";

import DataContainer, { DataContainerProps } from "./container";
import { IndexData, TableData } from "./types";

export type DataAdaptorProps = {
  dataContainers: DataContainer[];
} & DataContainerProps;

export default class DataAdaptor extends DataContainer {
  private dataContainers: DataContainer[];

  constructor(props: DataAdaptorProps) {
    super(props);

    const { dataContainers } = props;

    this.dataContainers = dataContainers;
  }

  use<T>(props: T): T {
    for (const prop in props) {
      const stateKey = this.bindStateKey[prop];
      if (stateKey) {
        this.bindProps[stateKey] = prop;
      }
    }

    const sliceDict: Record<string, any> = {};
    let queries = this.getQueries();
    sliceDict[this.name] = [0, queries.length];

    for (const container of this.dataContainers) {
      const _queries = container.getQueries();
      const start = queries.length;
      const end = queries.length + _queries.length;
      sliceDict[container.name] = [start, end];
      queries = queries.concat(_queries);
    }

    const results = useQueries(queries);
    const [start, end] = sliceDict[this.name];
    this.setResults(results.slice(start, end));
    
    for (const container of this.dataContainers) {
      const [start, end] = sliceDict[container.name];
      container.setResults(results.slice(start, end));
    }

    const newProps: any = {};
    for (const prop in props) {
      newProps[prop] = this.props[prop] || props[prop];
    }
    return newProps;
  }

  update(props: Record<string, any>): void {
    this.props = props;
  }

  getData(id: string): Data | undefined {
    for (const container of this.dataContainers) {
      const data = container.getData(id);
      if (data) {
        return data;
      }
    }
    return;
  }

  getProp<T>(propName:string): T | undefined {
    let propValue:T | undefined;
    for (const container of this.dataContainers) {
      propValue = container.getProp<T>(propName)
      break;
    }
    return propValue;
  }

  getProps(): Array<Record<string, any>> | undefined{
    const propsList:Array<Record<string, any>> = []
    for (const container of this.dataContainers) {
      const props = container.getProps()
      if(props && !Array.isArray(props))
        propsList.push(props)
    }
    return propsList
  }

  indexes(): IndexData[] {
    // let data: IndexData[] = [];
    // for (const container of this.dataContainers) {
    //   const items = container.useIndex();
    //   data = data.concat(items);
    //   break;
    // }
    // return data;
    return []
  }

  table(): TableData | undefined {
    let tables: TableData[] = [];
    for (const container of this.dataContainers) {
      const table = container.table();

      if (table) tables.push(table);
      break;
    }
    return;
  }
}
