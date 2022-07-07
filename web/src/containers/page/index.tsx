import React, { useMemo, useEffect } from "react";

import { useParams, useSearchParams } from "react-router-dom";
import { useQuery } from "react-query";

import Typography from "@mui/material/Typography";
import Box from "@mui/material/Box";

import { useWebSocket, getPage } from "@/services/api";
import { WidgetEvent } from "./types";
import createPage from "./create-page";

export default function WebPackage() {
  const params = useParams();
  const { webPackageName } = params;
  
  useEffect(() => {
    document.title = webPackageName;
  }, [])

  let [searchParams] = useSearchParams();
  if (webPackageName === undefined) return <></>;

  let path = params["*"] || "";
  path =
    searchParams.toString() !== ""
      ? path + "?" + searchParams.toString()
      : path;

  const result = useQuery<any, Error>([webPackageName, "page"], () =>
    getPage(webPackageName, path)
  );
  const [emit] = useWebSocket(webPackageName);

  const { status, data, error, isFetching } = result;

  const memo = useMemo(() => {
    if (data === undefined) return <></>;
    const handler = (event: WidgetEvent) => {
      emit(event);
    };
    return createPage(data, webPackageName, handler);
  }, [data]);

  return status === "loading" ? (
    <Box sx={{ pl: 2, pt: 1 }}>
      <Typography variant="overline">Loading...</Typography>
    </Box>
  ) : status === "error" ? (
    <Box sx={{ pl: 2, pt: 1 }}>
      <Typography variant="overline">Error: {error.message}</Typography>
    </Box>
  ) : (
    <div>{memo}</div>
  );
}
