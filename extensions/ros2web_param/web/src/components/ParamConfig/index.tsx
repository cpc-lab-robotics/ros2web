import React, { useEffect, useState } from "react";

import { Param, Range, PARAMETER } from "@/models/param";

import Table from "@mui/material/Table";
import TableBody from "@mui/material/TableBody";

import {
  SliderTableRow,
  SwitchTableRow,
  TextTableRow,
} from "./rows";

export type ParamConfigProps = {
  params: Array<Param>;
  on_change?: (param: Param) => void;
};

export default function ParamConfig(props: ParamConfigProps) {
  const { params: _params, on_change } = props;

  const [params, setParams] = useState<Array<Param>>(_params);

  useEffect(() => {
    setParams(_params);
  }, [_params]);

  const changeHandler = (value: any, index: number) => {
    if (params) {
      const p = [...params];
      p[index].value = value;
      setParams(p);
    }
  };

  const committedHandler = (param: Param) => {
    if (params) {
      if (on_change) on_change(param);
    }
  };

  return (
    <Table sx={{ p: 0 }} size="small">
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

                if (range !== undefined) {
                  return SliderTableRow(
                    key,
                    param.name,
                    Number(param.value),
                    range,
                    (value) => changeHandler(value, index),
                    () => committedHandler(param)
                  );
                } else {
                  return TextTableRow(
                    key,
                    param.name,
                    param.value,
                    "number",
                    (value) => changeHandler(value, index),
                    () => committedHandler(param)
                  );
                }
              } else if (param.descriptor.type === PARAMETER.BOOL) {
                return SwitchTableRow(
                  key,
                  param.name,
                  Boolean(param.value),
                  (value) => {
                    changeHandler(value, index);
                    committedHandler(param);
                  }
                );
              } else {
                return TextTableRow(
                  key,
                  param.name,
                  param.value,
                  "text",
                  (value) => changeHandler(value, index),
                  () => committedHandler(param)
                );
              }
            } else {
              return TextTableRow(
                key,
                param.name,
                param.value,
                "text",
                (value) => changeHandler(value, index),
                () => committedHandler(param)
              );
            }
          })}
      </TableBody>
    </Table>
  );
}
