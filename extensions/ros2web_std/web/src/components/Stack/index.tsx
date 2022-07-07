import React from "react";
import _Stack from "@mui/material/Stack";

export type StackProps = {
  direction?: "row" | "column" | "column-reverse" | "row-reverse";
  justify_content?: string;
  align_items?: string;
  spacing?: number;
};

type Props = {
  children?: React.ReactNode;
} & StackProps;

export default function Stack(props: Props) {
  const { spacing, direction, justify_content, align_items, children } = props;
  const stackProps = {
    ...(justify_content ? { justifyContent: justify_content } : {}),
    ...(align_items ? { alignItems: align_items } : {}),
    spacing: spacing || 1,
    direction,
  };
  return (
    <_Stack
      sx={{ height: "100%"}}
      {...stackProps}
    >
      {children}
    </_Stack>
  );
}
