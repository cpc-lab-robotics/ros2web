import { Dispatch } from "react";

import DataAdaptor from "@/containers/data/adaptor";

export type WidgetProps = {
  webPackageName?: string;
  bindStateKey?: Record<string, string>;
  initState?: Record<string, any>;
  setState?: Dispatch<Record<string, any>>;
  dataAdaptor?: DataAdaptor;
}

export type WidgetEvent = {
  event: {
    widget_id: string;
    type: string;
  }
  value?:any;
};
