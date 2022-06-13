import React, { useEffect } from "react";

import { styled } from "@mui/material/styles";
import Typography from "@mui/material/Typography";
import Box from "@mui/material/Box";
import Stack from "@mui/material/Stack";
import Table, { TableProps } from "@mui/material/Table";

import { Data } from "@/models";
import { WidgetProps, WidgetEvent } from "./types";
import { TableData } from "../data/types";

type TableWidgetProps = {
  title: string;
  on_selected?: (event: WidgetEvent) => void;
} & WidgetProps;

export default function TableWidget(props: TableWidgetProps) {
  const { dataAdaptor, on_selected, title } = props;

  let tableData: TableData[] = [];
  let tableProps: TableWidgetProps;
  if (dataAdaptor) {
    tableProps = dataAdaptor.use<TableWidgetProps>({
      title
    });

    // dataAdaptor.useIndex()
    
  }else{
    tableProps = {
      title
    }
  }

  console.log(tableProps)

  if (tableData.length === 0) {
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

  const selectedHandler = (item: any) => {
    let data: Data | undefined = undefined;
    if (dataAdaptor && item) {
      data = dataAdaptor.getData(item);
    }
    if (on_selected) {
      const event: WidgetEvent = {
        event: {
          widget_id: "__id__",
          type: "on_selected",
        },
        // value: (graphNode) ? graphNode.id : null,
      };
      on_selected(event);
    }
  };

  return (<Table>



  </Table>);
}
