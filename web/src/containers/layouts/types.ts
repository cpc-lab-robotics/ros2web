import { Dispatch } from "react";

export type LayoutProps = {
  webPackageName?: string;
  layout?: Record<string, any>;
  children?: React.ReactNode;
};

export type LayoutData = {
  style: string;
};
