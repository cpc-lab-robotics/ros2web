import React, { useEffect, useId, useState } from "react";

import { Param, Range, PARAMETER } from "../../models/param";

import Table from "@mui/material/Table";
import TableBody from "@mui/material/TableBody";
import TableCell from "@mui/material/TableCell";
import TableContainer from "@mui/material/TableContainer";
import TableHead from "@mui/material/TableHead";
import TableRow from "@mui/material/TableRow";
import Slider from "@mui/material/Slider";
import Switch from "@mui/material/Switch";
import TextField from "@mui/material/TextField";
import { string } from "yup";

function sliderTableRow(
  key: string,
  name: string,
  value: number,
  range: Range,
  onChange: (value:any) => void,
  onChangeCommitted: () => void
) {
  return (
    <TableRow key={key}>
      <TableCell
        component="th"
        scope="row"
        style={{
          whiteSpace: "nowrap",
          overflow: "hidden",
          textOverflow: "ellipsis",
          maxWidth: 100,
        }}
      >
        {name}
      </TableCell>
      <TableCell align="right" style={{ minWidth: 100 }}>
        <Slider
          defaultValue={value}
          step={range.step}
          min={range.fromValue}
          max={range.toValue}
          value={value}
          onChange={(event, value)=>onChange(value)}
          onChangeCommitted={onChangeCommitted}
        />
      </TableCell>

      <TableCell align="right" style={{ width: 100 }}>
        <TextField
          size="small"
          inputProps={{
            sx: {
              textAlign: "right",
            },
          }}
          value={value}
          variant="outlined"
          onChange={(event)=>{
            const v = Number(event.target.value);
            if(!isNaN(v)){
              if(range.fromValue <= v && v <= range.toValue)
                onChange(v)
            }
          }}
          onKeyDown={(event: any) => {
            if (event.keyCode === 13) {
              if (onChangeCommitted) onChangeCommitted();
            }
          }}
        />
      </TableCell>
    </TableRow>
  );
}
function switchTableRow(key: string, name: string, value: boolean) {
  return (
    <TableRow key={key}>
      <TableCell
        component="th"
        scope="row"
        style={{
          whiteSpace: "nowrap",
          overflow: "hidden",
          textOverflow: "ellipsis",
          maxWidth: 100,
        }}
      >
        {name}
      </TableCell>
      <TableCell colSpan={2} align="center">
        <Switch checked={value} />
      </TableCell>
    </TableRow>
  );
}

function textFieldTableRow(key: string, name: string, value: any) {
  return (
    <TableRow key={key}>
      <TableCell
        component="th"
        scope="row"
        style={{
          whiteSpace: "nowrap",
          overflow: "hidden",
          textOverflow: "ellipsis",
          maxWidth: 100,
        }}
      >
        {name}
      </TableCell>
      <TableCell colSpan={2} align="right">
        <TextField
          size="small"
          inputProps={{
            sx: {
              // textAlign: "right",
            },
          }}
          variant="outlined"
          fullWidth={true}
          value={value}
        />
      </TableCell>
    </TableRow>
  );
}

type Props = {
  params: Param[];
  onChangeCommitted?: (params: Param[]) => void;
};

export default function ParamEdit(props: Props) {
  const { params:_params, onChangeCommitted } = props;

  const [params, setParams] = useState<Array<Param>>(_params);
  
  useEffect(()=>{
    setParams(_params)
  }, [_params])

  const changeHandler = (value: any, index: number) => {

    if(params){
      const p = [...params];
      p[index].value = value
      setParams(p);
    }
    
  };

  const committedHandler = () => {
    if(params){
      if(onChangeCommitted)
        onChangeCommitted(params)
    }
  };

  return (
    <Table sx={{ p: 0 }} size="small">
      {/* <TableHead>
        <TableRow>
          <TableCell >name</TableCell>
          <TableCell align="right"></TableCell>
          <TableCell align="right"></TableCell>
        </TableRow>
      </TableHead> */}
      <TableBody>
        {params &&
          params.map((param, index) => {
            const key = `${param.name}-${index}`;
            if (param.descriptor) {
              if (
                param.descriptor.floatingPointRange ||
                param.descriptor.integerRange
              ) {
                const range: Range | undefined = param.descriptor
                  .floatingPointRange
                  ? param.descriptor.floatingPointRange
                  : param.descriptor.integerRange;

                if (range === undefined) return <TableRow key={key} />;

                return sliderTableRow(
                  key,
                  param.name,
                  Number(param.value),
                  range,
                  (event) => changeHandler(event, index),
                  committedHandler
                );
              } else if (param.descriptor.type === PARAMETER.BOOL) {
                return switchTableRow(key, param.name, Boolean(param.value));
              }
              return textFieldTableRow(key, param.name, param.value);
            } else {
              return textFieldTableRow(key, param.name, param.value);
            }
          })}
      </TableBody>
    </Table>
  );
}
