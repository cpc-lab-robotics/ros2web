import React from "react";
import _Masonry from "@mui/lab/Masonry";
import Box from "@mui/material/Box";

export type MasonryProps = {
  children:
    | string
    | number
    | boolean
    | React.ReactElement<any, string | React.JSXElementConstructor<any>>
    | React.ReactFragment
    | React.ReactPortal;
};

export default function Cards(props: MasonryProps) {
  const { children } = props;
  
  return (
    <Box sx={{ p: 1 }}>
      <_Masonry
        sx={{ m: 0, p: 0 }}
        spacing={1}
        columns={{ xs: 1, sm: 2, md: 3, lg: 4, xl: 5 }}
      >
        {children}
      </_Masonry>
    </Box>
  );
}
