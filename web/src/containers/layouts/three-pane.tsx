import React, { useState, useEffect } from "react";
import { useWindowSize, useGlobalState } from "@/hooks";

import ThreePane, { ThreePaneProps } from "@/components/ThreePane";
import { LayoutProps } from "./types";

export type ThreePaneLayoutProps = {};
type Props = ThreePaneLayoutProps & LayoutProps;

export default function ThreePaneLayout(props: Props) {
  const { children, webPackageName } = props;

  const windowSize = useWindowSize();
  const [resizeDate, setResizeDate] = useGlobalState<Date>(
    ["resizeEvent"],
    new Date()
  );
  
  useEffect(() => {
    setResizeDate(new Date());
  }, [windowSize]);

  const handleChangeSize = () => {
    setResizeDate(new Date());
  };
  
  const threePaneProps: ThreePaneProps = {
    name: webPackageName,
  };

  return (
    <ThreePane onChangeSize={handleChangeSize} {...threePaneProps}>
      {children}
    </ThreePane>
  );
}
