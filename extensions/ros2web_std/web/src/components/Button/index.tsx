import React from "react";
import Button from "@mui/material/Button";

import Stack, {StackProps} from "@/components/Stack";

type Props = {
  label?: string;
  labels?: string[];
  on_click?: (data: any) => void;
} & StackProps;

export default function ButtonWidget(props: Props) {
  const {
    on_click,
    label,
    labels,
    ...rest
  } = props;

  const clickHandler = async (data: any) => {
    if (on_click) {
      on_click(data);
    }
  };
  const buttonLabels = labels || [label || "Button"];
  
  return (
    <Stack
      {...rest}
    >
      {buttonLabels.map((buttonLabel, index) => (
        <Button
          key={index}
          onClick={() => clickHandler({
            index,
            label: buttonLabel
          })}
          variant="contained"
        >
          {buttonLabel}
        </Button>
      ))}
    </Stack>
  );
}
