import React from "react";
import _Box from "@mui/material/Box";

export type BoxProps = {
  justify_content?: string;
  align_items?: string;
};

type Props = {
  children?: React.ReactNode;
} & BoxProps;

export default function Box(props: Props) {
  const { justify_content, align_items, children } = props;
  const boxProps = {
    ...(justify_content ? { justifyContent: justify_content } : {justifyContent:"center"}),
    ...(align_items ? { alignItems: align_items } : {alignItems:"center"}),
  };
  return (
    <_Box
      sx={{ height: "100%", width: "100%", display:"flex"}}
      {...boxProps}
    >
      {children}
    </_Box>
  );
}
