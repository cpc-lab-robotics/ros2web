import React from "react";

import { styled } from "@mui/material/styles";
import Typography from "@mui/material/Typography";
import Grid from "@mui/material/Grid";
import Stack from "@mui/material/Stack";

import { PackageManifest } from "../../models/package";
import { Box } from "@mui/material";

import TabPanel from "../TabPanel";
import PackageManifestEdit from "./edit";
import PackageManifestView from "./view";

export type PackageManifestProps = {
  packageManifest?: PackageManifest;
  editable?: boolean;
};

export default function PackageManifestComponent(props: PackageManifestProps) {
  const { packageManifest, editable} = props;
  
  if (packageManifest === undefined || packageManifest === null) {
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
            Not selected.
          </Typography>
        </Stack>
      </Box>
    );
  }

  return (
    <React.Fragment>
      <TabPanel value={true} tabValue={editable}>
        <PackageManifestEdit packageManifest={packageManifest} />
      </TabPanel>
      <TabPanel value={false} tabValue={editable}>
        <PackageManifestView packageManifest={packageManifest}/>
      </TabPanel>
    </React.Fragment>
  );
}
