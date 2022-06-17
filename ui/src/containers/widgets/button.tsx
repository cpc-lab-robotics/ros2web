import React from "react";
import Button, { ButtonProps } from "@mui/material/Button";
import { WidgetProps, WidgetEvent } from "./types";

type ButtonWidgetProps = {
  label?: string;
  variant?: "contained" | "outlined" | "text";
  disabled?: boolean;
  on_click?: (event: WidgetEvent) => void;
} & WidgetProps;

export default function ButtonWidget(_props: ButtonWidgetProps) {
  const {
    dataAdaptor,
    on_click,
    ...rest
  } = _props;

  let props: ButtonWidgetProps = {
    label: undefined,
    variant: 'contained',
    disabled: false,
    ...rest,
  };
  
  if (dataAdaptor) {
    props = dataAdaptor.use<ButtonWidgetProps>(props);
  }
  const { label, variant, disabled } = props;
  
  const clickHandler = (event: React.MouseEvent) => {
    if (on_click) {
      const event: WidgetEvent = {
        event: {
          widget_id: "__id__",
          type: "on_click",
        },
      };
      on_click(event);
    }
  };
  const buttonProps: ButtonProps = {
    variant,
    disabled,
    onClick: clickHandler,
  };
  
  return <Button {...buttonProps}>{label}</Button>;
}
