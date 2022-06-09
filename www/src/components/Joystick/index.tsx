import { Dispatch } from "react";
import { Joystick } from "react-joystick-component";
import {
  IJoystickProps,
  IJoystickUpdateEvent,
} from "react-joystick-component/build/lib/Joystick";

import { useTheme } from "@mui/material";
import { styled } from "@mui/material/styles";

export type JoystickProps = {
  base_color?: string;
  stick_color?: string;
};

type Props = JoystickProps;

export default function JoystickWidget(props: Props) {
  const {
    base_color,
    stick_color,
  } = props;

  const theme = useTheme();

  const baseColor =
    base_color !== undefined ? base_color : theme.palette.secondary.main;
  const stickColor =
    stick_color !== undefined ? stick_color : theme.palette.primary.main;

  const startHandler = (jev: IJoystickUpdateEvent) => {
    
  };
  
  const moveHandler = (jev: IJoystickUpdateEvent) => {
    const data = {
      direction: jev.direction,
      distance: jev.distance,
      x: jev.x,
      y: jev.y,
    }

  };

  const stopHandler = (jev: IJoystickUpdateEvent) => {
    
  };

  return (
    <Joystick
            baseColor={baseColor}
            stickColor={stickColor}
            start={startHandler}
            move={moveHandler}
            stop={stopHandler}
          />
  );
}