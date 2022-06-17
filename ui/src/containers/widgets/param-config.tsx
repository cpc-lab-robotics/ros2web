import React, { useEffect } from "react";

import { styled } from "@mui/material/styles";
import Typography from "@mui/material/Typography";
import Box from "@mui/material/Box";
import Stack from "@mui/material/Stack";

import ParamConfig, {ParamConfigProps} from "@/components/ParamConfig";
import { Data } from "@/models";
import { WidgetProps, WidgetEvent } from "./types";
import { Param } from "@/models/param";

type ParamConfigWidgetProps = {
  params: Param[];
  on_change?: (event: WidgetEvent) => void;
} & WidgetProps;

export default function ParamConfigWidget(props: ParamConfigWidgetProps) {
  const { dataAdaptor, params: _params, on_change } = props;

  let paramConfigWidgetProps: ParamConfigWidgetProps;
  if (dataAdaptor) {
    paramConfigWidgetProps = dataAdaptor.use<ParamConfigWidgetProps>({
      params: _params,
    });

    // dataAdaptor.useIndex()
  } else {
    paramConfigWidgetProps = {
      params: _params,
    };
  }

  const { params } = paramConfigWidgetProps;

  if (params === undefined || params === null || params.length === 0) {
    return (
      <Box sx={{ p: 3, width: "100%", height: "100%" }}>
        <Stack
          sx={{ height: "100%" }}
          justifyContent="center"
          alignItems="center"
        >
          <Typography variant="overline" color="text.secondary">
            Not data.
          </Typography>
        </Stack>
      </Box>
    );
  }

  const changeHandler = (param: Param) => {
    if (on_change) {
      const event: WidgetEvent = {
        event: {
          widget_id: "__id__",
          type: "on_change",
        },
        value: param
      };
      on_change(event);
    }
  };

  const paramConfigProps: ParamConfigProps = {
    params,
    onChangeCommitted: changeHandler
  }
  return <ParamConfig {...paramConfigProps}/>;
}
