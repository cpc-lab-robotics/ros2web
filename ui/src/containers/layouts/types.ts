import { Dispatch } from "react";

export type LayoutProps = {
  webPackageName?: string;
  bindStateKey?: Record<string, string>;
  initState?: Record<string, any>;
  setState?: Dispatch<Record<string, any>>;
  children?: React.ReactNode;
}

// export const LAYOUT_STYLE = {
//   FLAT: 'LAYOUT_STYLE_FLAT',
//   CARD: 'LAYOUT_STYLE_CARD',
// } as const;
// export type LayoutStyle = typeof LAYOUT_STYLE[keyof typeof LAYOUT_STYLE];
