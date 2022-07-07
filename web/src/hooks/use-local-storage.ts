import React, { Dispatch, SetStateAction } from "react";


export function useLocalStorage<T>(
  storageKey:string,
  fallbackState?: T
): [T, Dispatch<SetStateAction<T>>] {

  const [state, setState] = React.useState<T>(
    JSON.parse(localStorage.getItem(storageKey)) ?? fallbackState
  );
  
  React.useEffect(() => {
    if(state !== undefined)
      localStorage.setItem(storageKey, JSON.stringify(state));
  }, [state, storageKey]);
  return [state, setState];
}