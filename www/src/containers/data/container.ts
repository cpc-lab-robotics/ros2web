import { Dispatch } from "react";
import { getStateValue } from "@/services/api";
import { Data } from "@/models";
import { IndexData, TableData } from "./types";
import { string } from "yup";

export type DataContainerProps = {
  webPackageName: string;
  bindStateKey: Record<string, string>;
  initState: Record<string, any>;
  setState: Dispatch<Record<string, any>>;
};

export default abstract class DataContainer {
  protected webPackageName: string;
  protected bindStateKey: Record<string, string>;
  protected initState: Record<string, any>;
  protected setState: Dispatch<Record<string, any>>;
  protected bindProps: Record<string, string>;
  protected props: Record<string, any>;

  constructor(props: DataContainerProps) {
    const { webPackageName, initState, bindStateKey, setState, ...rest} = props;
    this.webPackageName = webPackageName;
    this.initState = initState;
    this.bindStateKey = bindStateKey;
    this.setState = setState;

    this.bindProps = {};
    this.props = rest;
  }

  getQueries() {
    return Object.keys(this.bindProps).map((key: string) => ({
      queryKey: [this.webPackageName, "state", key],
      queryFn: () => getStateValue(this.webPackageName, key),
      initialData: this.initState[key],
    }));
  }

  setResults(results: any[]) {
    const props: Record<string, any> = {};

    Object.keys(this.bindProps).map((key: string, index) => {
      const { data } = results[index];
      const propName = this.bindProps[key];
      props[propName] = data;
    });

    this.update(props);
  }
  get name(): string {
    return this.constructor.name;
  }

  protected abstract update(props: Record<string, any>): void;
  abstract getData(id: string): Data | undefined;

  abstract getProp<T>(propName:string): T | undefined;
  abstract getProps(): Array<Record<string, any>> | Record<string, any> | undefined;
  abstract indexes(): IndexData[];
  abstract table(): TableData | undefined;
}
