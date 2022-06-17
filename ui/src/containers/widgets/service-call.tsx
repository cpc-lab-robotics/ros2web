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
import { TableData } from "../data/types";
import { Service } from "@/models/service";
import { Field } from "@/models/interface";

type ServiceCallWidgetProps = {
  service: Service;
  on_selected?: (event: WidgetEvent) => void;
} & WidgetProps;

function fieldToRow(tabs: any[], name: string, fields: Field[]){

  let rows: string[][] = [];
  tabs.push({
    name,
    rows
  })

  for (const field of fields) {
    const column: Array<any> = [field.name, field.type];
    rows.push(column);
    if (field.nested) {
      fieldToRow(tabs, field.type, field.nested);
    }
  }
  
}

export default function ServiceCallWidget(props: ServiceCallWidgetProps) {
  const { dataAdaptor, on_selected, service } = props;

  let tableProps: ServiceCallWidgetProps;
  let tableRow: string[][] = [];
  if (dataAdaptor) {
    const { service: srv } = dataAdaptor.use<ServiceCallWidgetProps>({
      service,
    });
    
    if (srv && srv.descriptor) {
      const requestFields = srv.descriptor.request || [];
      const responseFields = srv.descriptor.response || [];
      const tabs: any[] = [];
      fieldToRow(tabs, 'response', responseFields);
    }
  } else {
    tableProps = {
      service,
    };
  }

  if (tableRow.length === 0) {
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

  return (
    <Table>
      {tableRow &&
        tableRow.map((row) => (
          <TableRow>
            {row.map((value) => (
              <TableCell align="right">{value}</TableCell>
            ))}
          </TableRow>
        ))}
    </Table>
  );
}
