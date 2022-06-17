import DataContainer, { DataContainerProps } from "./container";

import { Data } from "@/models";
import { IndexData, TableData } from "./types";

export type FlexibleDataContainerProps = {} & DataContainerProps;

export default class FlexibleDataContainer extends DataContainer {
  constructor(props: FlexibleDataContainerProps) {
    super(props);
    
    if (this.bindStateKey) {
      for (const prop in this.bindStateKey) {
        const stateKey = this.bindStateKey[prop];
        if (stateKey) {
          this.bindProps[stateKey] = prop;
        }
      }
    }
  }

  update(props: Record<string, any>): void {
    this.props = {...this.props, ...props};
  }
  
  getData(id: string): Data | undefined {
    return;
  }

  getProp<T>(propName:string): T | undefined {
    return this.props[propName]
  }

  getProps(): Record<string, any> | undefined{
    return this.props
  }

  indexes(): IndexData[] {
    return [];
  }

  table(): TableData | undefined {
    return;
  }

  // useInfo(): InfoData | undefined {
  //   this.update();

  //   const items: Array<KeyValueWithHeader | KeyValue> = [];
  //   for (const [key, value] of Object.entries(this.dict)) {
  //     const header = undefined;
  //     if (typeof value === "object") {
  //       const header = key;
  //       const keyValues: Array<KeyValue> =  []
  //       for (const [key2, value2] of Object.entries(value)) {
  //         keyValues.push({
  //           key: key2,
  //           value: value2,
  //         })
  //       }
  //       items.push({
  //         header,
  //         keyValues,
  //       });
  //     } else {
  //       const keyValue: KeyValue = {
  //         key,
  //         value,
  //       };
  //       items.push(keyValue);
  //     }
  //   }

  //   return {
  //     title: "",
  //     items,
  //   };
  // }
}
