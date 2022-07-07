import React from "react";
import { useQueries, UseQueryResult } from "react-query";
import { getStateValue } from "@/services/api";

export function useStateKey(
  webPackageName: string,
  initialState: Record<string, any>
): Record<string, any> {
  const state: Record<string, any> = {};

  const keys = Object.keys(initialState);
  const results = useQueries(
    keys.map((key: string) => ({
      queryKey: [webPackageName, "state", key],
      queryFn: () => getStateValue(webPackageName, key),
      initialData: initialState[key],
    }))
  );
  keys.forEach((key, index) => {
    const { data } = results[index];
    state[key] = data;
  });
  return state;
}
