import React, { useId } from "react";

import Typography from "@mui/material/Typography";
import Box from "@mui/material/Box";
import Stack from "@mui/material/Stack";

import { Param, Range, PARAMETER } from "@/models/param";
import TabPanel from "@/components/TabPanel";

import PramEdit from "./edit";
import PramView from "./view";

export type ParamProps = {
  params?: Param[];
  editable?: boolean;
};

export default function ParamComponent(props: ParamProps) {
  const { editable, params, ...rest } = props;
  
  if (params === undefined || params === null) {
    return (
      <Box sx={{ p: 2, width: "100%", height: "100%"}}>
        <Stack
          sx={{height: "100%"}}
          justifyContent="center"
          alignItems="center"
        >
          <Typography
            variant="overline"
            color="text.secondary"
          >
            Not data.
          </Typography>
        </Stack>
      </Box>
    );
  }

  return (
    <React.Fragment>
      <TabPanel value={true} tabValue={editable}>
        <PramEdit params={params} {...rest} />
      </TabPanel>
      <TabPanel value={false} tabValue={editable}>
        <PramView params={params} {...rest}/>
      </TabPanel>
    </React.Fragment>
  );
}
