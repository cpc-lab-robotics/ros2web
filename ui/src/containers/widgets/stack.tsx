import React from "react";

import Stack from "@mui/material/Stack";
import Divider from '@mui/material/Divider';

import { WidgetProps, WidgetEvent } from "./types";

type StackWidgetProps = {
  justify_content?:string;
  align_items?: string;
  direction?: "column" | "row" | "row-reverse" | "column-reverse";
  children?: React.ReactNode;
} & WidgetProps;

export default function StackWidget(_props: StackWidgetProps) {
  const { dataAdaptor, children, ...rest } = _props;

  let props: StackWidgetProps = {
    justify_content: 'space-around',
    align_items: 'center',
    direction: "column",
    ...rest,
  };
  if (dataAdaptor) {
    props = dataAdaptor.use<StackWidgetProps>(props);
  }
  const { direction, justify_content, align_items} = props;
  
  // const cards: React.ReactElement[] = [];
  // const childElements = React.Children.toArray(children);
  // for (const childElement of childElements) {
  //   if (React.isValidElement(childElement)) {
  //   }
  // }
  return (
    <Stack
      direction={direction}
      // divider={<Divider orientation="vertical" flexItem />}
      sx={{ minHeight: 60 }}
      justifyContent={justify_content}
      alignItems={align_items}
      spacing={2}
    >
      {children}
    </Stack>
  );
}
