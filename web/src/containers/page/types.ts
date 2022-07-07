import { Dispatch } from "react";

export interface WebPackage {
  name: string;
  packageName: string;
  active: boolean;
}

export type WidgetEvent = {
  event: {
    widget_id: string;
    type: string;
  };
  value?: any;
};

export type LayoutProps = {
  webPackageName?: string;
  bindStateKey?: Record<string, string>;
  initState?: Record<string, any>;
  setState?: Dispatch<Record<string, any>>;
  children?: React.ReactNode;
};
