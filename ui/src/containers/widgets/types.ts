import { Dispatch } from "react";
import DataAdaptor from "../data/adaptor";


export type WidgetProps = {
  dataAdaptor?: DataAdaptor;
}

export type WidgetEvent = {
  event: {
    widget_id: string;
    type: string;
  }
  value?:any;
};
