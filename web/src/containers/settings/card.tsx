import React, { useState } from "react";

import { useNavigate } from "react-router-dom";
import { useMutation, useQuery, useQueryClient } from "react-query";

import Card from "@mui/material/Card";
import CardActions from "@mui/material/CardActions";
import CardContent from "@mui/material/CardContent";
import CardHeader from "@mui/material/CardHeader";
import Button from "@mui/material/Button";
import Typography from "@mui/material/Typography";
import Switch from "@mui/material/Switch";
import Box from "@mui/material/Box";

import { Plugin, getPlugin, enablePlugin, disablePlugin } from "@/services/api";

function usePlugin(pluginId: string) {
  return useQuery<Plugin, Error>(["ros2web", "plugin", pluginId], () =>
    getPlugin(pluginId)
  );
}

function usePluginState() {
  const queryClient = useQueryClient();
  const enable = useMutation((pluginId: string) => enablePlugin(pluginId), {
    onSuccess: (plugin) => {
      queryClient.setQueryData<Plugin>(
        ["ros2web", "plugin", plugin.id],
        plugin
      );
    },
  });

  const disable = useMutation((pluginId: string) => disablePlugin(pluginId), {
    onSuccess: (plugin) => {
      queryClient.setQueryData<Plugin>(
        ["ros2web", "plugin", plugin.id],
        plugin
      );
      
      // if(plugin.disable && window[plugin.name]){
      //   const p = window[plugin.name]
      //   console.log(p)
      // }
      

    },
  });

  return [enable, disable];
}

type CardColor = {
  body: string;
  title: string;
  summary: string;
};

export type PluginCardProps = {
  pluginId: string;
};

export default function PluginCard(props: PluginCardProps) {
  const { pluginId } = props;

  // const navigate = useNavigate();

  const [enable, disable] = usePluginState();
  const switchHandler = (event: React.ChangeEvent<HTMLInputElement>) => {

    if (event.target.checked) {
      enable.mutate(plugin.id);
    } else {
      disable.mutate(plugin.id);
    }
  };

  const clickHandler = (name: string) => {
    // navigate(`/ros2web/${name}`);
    window.open(`/ros2web/${name}`, "_blank");
  };

  const { status, data: plugin, error } = usePlugin(pluginId);

  return status === "loading" ? (
    <Box sx={{ pl: 2, pt: 1 }}>
      <Typography variant="overline">Loading...</Typography>
    </Box>
  ) : status === "error" ? (
    <Box sx={{ pl: 2, pt: 1 }}>
      <Typography variant="overline">Error: {error.message}</Typography>
    </Box>
  ) : (
    (() => {
      const color: CardColor = {
        body:
          plugin.type === "package"
            ? plugin.disable
              ? "rgba(255,255,255,0.5)"
              : "#fff"
            : plugin.disable
            ? "rgba(35, 50, 79, 0.1)"
            : "rgba(35, 50, 79, 0.5)",
        title: plugin.type === "package" ? "text.primary" : "#eee",
        summary: plugin.type === "package" ? "#555" : "#ddd",
      };

      return (
        <Card sx={{ minWidth: 50, bgcolor: color.body }}>
          <CardHeader
            title={plugin.name}
            titleTypographyProps={{
              variant: "overline",
              fontWeight: "bold",
              color: color.title,
            }}
            sx={{
              userSelect: "none",
              height: 40,
            }}
          />
          <CardContent sx={{ pt: 0, pb: 1 }}>
            <Typography variant={"body2"} color={color.summary}>
              {plugin.summary}
            </Typography>
          </CardContent>
          <CardActions disableSpacing>
            {plugin.type === "package" && (
              <Button
                variant="outlined"
                onClick={() => clickHandler(plugin.name)}
                disabled={plugin.disable}
              >
                Open
              </Button>
            )}
            <Switch
              sx={{ marginLeft: "auto" }}
              checked={!plugin.disable}
              onChange={switchHandler}
            />
          </CardActions>
        </Card>
      );
    })()
  );
}
