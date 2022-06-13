import { styled } from "@mui/material/styles";
import Typography from "@mui/material/Typography";
import Grid from "@mui/material/Grid";
import Stack from "@mui/material/Stack";

import { PackageManifest } from "../../models/package";
import { Box } from "@mui/material";


type Props = {
  packageManifest: PackageManifest;
};

export default function PackageManifestEdit(props: Props) {
  const { packageManifest } = props;
  return (
    <Box sx={{ p: 2 }}>
    </Box>
  );
}
