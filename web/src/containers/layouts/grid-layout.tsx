import React, { useMemo, useEffect, useState, ReactNode } from "react";

import { styled } from "@mui/material/styles";

import Grid, { GridItem, GridProps } from "@/components/Grid";
import { LayoutProps } from "./types";
import { useLocalStorage } from "@/hooks";

export type GridLayoutProps = {};
type Props = GridLayoutProps & LayoutProps;

function generateGridItem(
  elements: ReactNode[],
  layout: Record<string, any>
): GridItem[] {
  const items: GridItem[] = [];

  for (const element of elements) {
    if (React.isValidElement(element)) {
      const widgetKey = element.props["widgetKey"];
      const grid = layout[widgetKey];
      items.push({
        x: 0,
        y: 0,
        w: 2,
        h: 4,
        minW: 1,
        minH: 1,
        ...(grid ? grid : {}),
        i: String(element.key),
        static: true,
        element,
      });
    }
  }
  return items;
}

export default function GridLayout(props: Props) {
  const { children, webPackageName, layout: initLayout } = props;

  let childElements = [];
  if (children) {
    childElements = React.Children.toArray(children);
  }

  const [layout, setLayout] = useLocalStorage<Record<string, any>>(
    `${webPackageName}-layout`, {}
  );
  
  const gridItems = React.useMemo(() => {
    const updateGrid = {};
    for (const [key, initGrid] of Object.entries(initLayout)) {
      const grid = layout[key] || {};
      updateGrid[key] = {
        ...initGrid,
        ...grid,
      };
    }
    return generateGridItem(childElements, updateGrid);
  }, [childElements, layout, initLayout]);

  const onLayoutChange = (items: GridItem[]) => {
    const layout = {};
    for (const item of items) {
      const widgetKey = item.element.props["widgetKey"];
      layout[widgetKey] = {
        x: item.x,
        y: item.y,
        w: item.w,
        h: item.h,
      };
    }
    setLayout(layout);
  };

  return (
    <Grid
      onLayoutChange={onLayoutChange}
      initialLayout={gridItems}
      title={webPackageName}
    />
  );
}
