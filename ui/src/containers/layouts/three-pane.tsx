import { useState, useEffect } from "react";
import { useQueries } from "react-query";
import { useProps } from "@/services/api";
import { useWindowSize, useGlobalState } from "@/hooks";

import ThreePane, { ThreePaneProps } from "@/components/ThreePane";
import { LayoutProps } from "./types";

export type ThreePaneLayoutProps = {
  name: string;
};
type Props = ThreePaneLayoutProps & LayoutProps;

export default function ThreePaneLayout(props: Props) {
  const { children, webPackageName, bindStateKey, initState, setState, name } =
    props;

  const windowSize = useWindowSize();
  const [resizeDate, setResizeDate] = useGlobalState<Date>(
    ["resizeEvent"],
    new Date()
  );

  useEffect(() => {
    setResizeDate(new Date());
  }, [windowSize]);

  let updateProps: Record<string, any> = {}
  if (webPackageName && initState && bindStateKey) {
    const bindProps: Record<string, string> = {};
    for (const prop in ["name"]){
      const stateKey = bindStateKey[prop];
      if(stateKey) {
        bindProps[stateKey] = prop;
      }
    }
    updateProps = useProps(
      webPackageName,
      initState,
      bindProps
    );
  }

  const handleChangeSize = () => {
    setResizeDate(new Date());
  };
  
  const threePaneProps: ThreePaneProps = {
    name: updateProps["name"] || name,
  };

  return (
    <ThreePane onChangeSize={handleChangeSize} {...threePaneProps}>
      {children}
    </ThreePane>
  );
}
