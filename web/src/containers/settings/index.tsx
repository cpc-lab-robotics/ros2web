import React from "react";
import { useQuery } from "react-query";

import { useTheme, styled } from "@mui/material/styles";
import Grid from "@mui/material/Grid";
import Divider from "@mui/material/Divider";
import AppBar from "@mui/material/AppBar";
import Box from "@mui/material/Box";
import Toolbar from "@mui/material/Toolbar";
import Typography from "@mui/material/Typography";

import { getPlugins } from "@/services/api";

import PluginCard from "./card";

const Header = styled("div")(({ theme }) => ({
  ...theme.mixins.toolbar,
}));

export default function Settings() {
  const theme = useTheme();
  const title = "ros2web";

  const result = useQuery<Array<string>, Error>(
    ["ros2web", "plugins"],
    getPlugins
  );

  const { status, data, error } = result;

  return status === "loading" ? (
    <Box sx={{ pl: 2, pt: 1 }}>
      <Typography variant="overline">Loading...</Typography>
    </Box>
  ) : status === "error" ? (
    <Box sx={{ pl: 2, pt: 1 }}>
      <Typography variant="overline">Error: {error.message}</Typography>
    </Box>
  ) : (
    <Box
      sx={{
        bgcolor: theme.palette.secondary.main,
        display: "flex",
        height: "100vh",
      }}
    >
      <AppBar color="inherit" position="fixed" elevation={1}>
        <Toolbar variant="dense">
          <Typography variant="overline" sx={{userSelect: "none", flexGrow: 1}}>{title}</Typography>
        </Toolbar>
      </AppBar>

      <Box sx={{ flexGrow: 1 }}>
        <Header />
        <Grid
          container
          direction="row"
          justifyContent="flex-start"
          alignItems="flex-start"
          // bgcolor={'red'}
          spacing={2}
          sx={{ pl: 2, pr: 2 }}
        >
          <Grid item xs={12}>
            <Grid
              container
              direction="row"
              justifyContent="flex-start"
              alignItems="flex-start"
              // bgcolor={'red'}
              spacing={2}
            >
              {data
                .filter((pluginId) => pluginId.startsWith("package"))
                .map((packageId) => (
                  <Grid key={packageId} item xs={4}>
                    <PluginCard pluginId={packageId} />
                  </Grid>
                ))}
            </Grid>
          </Grid>
          <Grid item xs={12}>
            <Divider />
          </Grid>
          <Grid item xs={12}>
            <Grid
              container
              direction="row"
              justifyContent="flex-start"
              alignItems="flex-start"
              // bgcolor={'red'}
              spacing={2}
            >
              {data
                .filter((pluginId) => pluginId.startsWith("extension"))
                .map((extensionId) => (
                  <Grid key={extensionId} item xs={4}>
                    <PluginCard pluginId={extensionId} />
                  </Grid>
                ))}
            </Grid>
          </Grid>
        </Grid>
      </Box>
    </Box>
  );
}
