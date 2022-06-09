import React, { useState, useEffect } from "react";
import { useProps } from "@/services/api";
import { useWindowSize, useGlobalState } from "@/hooks";

import Masonry, { MasonryProps } from "@/components/Masonry";
import { LayoutProps } from "./types";

export type CardLayoutProps = {
  
};
type Props = CardLayoutProps & LayoutProps;

export default function CardLayout(props: Props) {
  const { children, webPackageName, initState, bindStateKey, setState, ...rest } =
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
    // for (const prop in ["name"]){
    //   const stateKey = bindStateKey[prop];
    //   if(stateKey) {
    //     bindProps[stateKey] = prop;
    //   }
    // }
    // updateProps = useProps(
    //   webPackageName,
    //   initState,
    //   bindProps
    // );
  }
  
  if (React.isValidElement(children)){
    const masonryProps: MasonryProps = {
      children
    };
    return <Masonry {...masonryProps}/>
  }else{
    return <></>
  }
}
