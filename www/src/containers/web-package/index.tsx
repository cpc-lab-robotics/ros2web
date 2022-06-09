import { useMemo, useEffect } from "react";

import { useParams } from "react-router-dom";
import { useWebPackageState } from "@/services/api";
import createLayout from "../create-layout";

import { WidgetEvent } from "@/containers/widgets/types";
import { UIData } from "./types";

export default function WebPackage() {
  const { webPackageName } = useParams();
  if (webPackageName === undefined) return <></>;

  const [info, state, setState, emit] = useWebPackageState(webPackageName);

  const memo = useMemo(() => {
    if (
      info === undefined ||
      state === undefined ||
      info === null ||
      state === null ||
      info.ui === undefined
    )
      return <></>;
    const ui: UIData = info.ui || {};
    const handler = (event: WidgetEvent) => {
      emit(event);
    };
    if (ui.element !== undefined) {
      return createLayout(
        webPackageName,
        ui.element,
        ui.bind,
        state,
        setState,
        handler
      );
    } else {
      return <></>;
    }
  }, [info, state]);

  return memo;
}
