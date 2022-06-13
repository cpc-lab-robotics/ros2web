import { Dispatch } from "react";

import { WidgetEvent, WidgetProps } from "./types";

import { Joystick } from "react-joystick-component";
import {
  IJoystickProps,
  IJoystickUpdateEvent,
} from "react-joystick-component/build/lib/Joystick";

import { useTheme } from "@mui/material";
import { styled } from "@mui/material/styles";
import Stack from "@mui/material/Stack";

export type JoystickWidgetProps = {
  on_start?: (event: WidgetEvent) => void;
  on_move?: (event: WidgetEvent) => void;
  on_stop?: (event: WidgetEvent) => void;
};

type Props = JoystickWidgetProps & WidgetProps;

export default function JoystickWidget(_props: Props) {
  const theme = useTheme();

  const { on_start, on_move, on_stop, dataAdaptor } = _props;

  let props: JoystickWidgetProps = {};
  if (dataAdaptor) {
    props = dataAdaptor.use<JoystickWidgetProps>(props);
  }
  const {} = props;

  const startHandler = (jev: IJoystickUpdateEvent) => {
    if (on_start) {
      const event: WidgetEvent = {
        event: {
          widget_id: "__id__",
          type: "on_start",
        },
      };
      on_start(event);
    }
  };
  const moveHandler = (jev: IJoystickUpdateEvent) => {
    if (on_move) {
      const event: WidgetEvent = {
        event: {
          widget_id: "__id__",
          type: "on_move",
        },
        value: {
          direction: jev.direction,
          distance: jev.distance,
          x: jev.x,
          y: jev.y,
        },
      };

      on_move(event);
    }
  };

  const stopHandler = (jev: IJoystickUpdateEvent) => {
    if (on_stop) {
      const event: WidgetEvent = {
        event: {
          widget_id: "__id__",
          type: "on_stop",
        },
      };
      on_stop(event);
    }
  };

  return (
    <Stack
      sx={{
        minHeight: 150,
      }}
      direction="row"
      justifyContent="center"
      alignItems="center"
    >
      <Joystick
        baseColor={theme.palette.secondary.main}
        stickColor={theme.palette.primary.main}
        start={startHandler}
        move={moveHandler}
        stop={stopHandler}
      />
    </Stack>
  );
}
