import React from "react";

import { Param, Range, PARAMETER } from "../../models/param";

import Table from "@mui/material/Table";
import TableBody from "@mui/material/TableBody";
import TableCell from "@mui/material/TableCell";
import TableHead from "@mui/material/TableHead";
import TableRow from "@mui/material/TableRow";

type Props = {
  params: Param[];
};

export default function ParamView(props: Props) {
  const { params } = props;
  
  return (
    <Table sx={{ p: 0 }} size="small" aria-label="simple table">
      <TableHead>
        <TableRow>
          <TableCell>name</TableCell>
          <TableCell align="right"></TableCell>
          <TableCell align="right"></TableCell>
        </TableRow>
      </TableHead>
      <TableBody>
      </TableBody>
    </Table>
  );
}