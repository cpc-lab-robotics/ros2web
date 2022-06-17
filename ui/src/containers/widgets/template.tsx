import React, { useEffect } from "react";

import { styled } from "@mui/material/styles";
import Typography from "@mui/material/Typography";
import Box from "@mui/material/Box";
import Stack from "@mui/material/Stack";
import Table, { TableProps } from "@mui/material/Table";
import TableRow from "@mui/material/TableRow";
import TableCell from "@mui/material/TableCell";

import { Data } from "@/models";
import { WidgetProps, WidgetEvent } from "./types";

type _WidgetProps = {
  on_selected?: (event: WidgetEvent) => void;
} & WidgetProps;


export default function _Widget(_props: _WidgetProps) {
  const { dataAdaptor, on_selected } = _props;

  let props: _WidgetProps;
  if (dataAdaptor) {
    props = dataAdaptor.use<_WidgetProps>({
      
    });
  } else {
    props = {
    };
  }

  const {} = props;
  
  // const selectedHandler = (item: any) => {
  //   let data: Data | undefined = undefined;
  //   if (dataAdaptor && item) {
  //     data = dataAdaptor.getData(item);
  //   }
  //   if (on_selected) {
  //     const event: WidgetEvent = {
  //       event: {
  //         widget_id: "__id__",
  //         type: "on_selected",
  //       },
  //       // value: (graphNode) ? graphNode.id : null,
  //     };
  //     on_selected(event);
  //   }
  // };

  return (
    <Stack>

    </Stack>
  );
}
