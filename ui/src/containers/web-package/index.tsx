import { useMemo, useEffect } from "react";

import { useParams, useSearchParams} from "react-router-dom";
import { useWebPackageState } from "@/services/api";

import { WidgetEvent } from "@/containers/widgets/types";

import createPage from "./create-page";


export default function WebPackage(){
  const params = useParams();
  const { webPackageName } = params;

  let [searchParams] = useSearchParams();
  if (webPackageName === undefined) return <></>;
  
  let path = params["*"] || '';
  path = path +'?'+ searchParams.toString()
  const [page, state, setState, emit] = useWebPackageState(webPackageName, path);

  const memo = useMemo(() => {
    if (
      page === undefined ||
      state === undefined
    )
      return <></>;
    
    const handler = (event: WidgetEvent) => {
      emit(event);
    };
    
    return createPage(
      page,
      webPackageName,
      state,
      setState,
      handler
    );
    
  }, [page, state]);

  return memo;
}
