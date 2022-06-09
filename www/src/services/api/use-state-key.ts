import React, { Dispatch, SetStateAction } from "react";

import { useQueries } from "react-query";
import { getStateValue } from "@/services/api";

export function useProps(
  webPackageName: string,
  initialState: Record<string, any>,
  bindProps: Record<string, string>
): Record<string, any> {
  const props: Record<string, any> = {};

  const results = useQueries(
    Object.keys(bindProps).map((key: string) => ({
      queryKey: [webPackageName, "state", key],
      queryFn: () => getStateValue(webPackageName, key),
      initialData: initialState[key],
    }))
  );
  
  Object.keys(bindProps).map((key: string, index) => {
    const { data } = results[index];
    const propName = bindProps[key];
    props[propName] = data;
  });
  return props;
}
