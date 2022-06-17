import React, { useRef, useCallback } from "react";

import { styled } from "@mui/material/styles";
import _Box, { BoxProps } from "@mui/material/Box";

type AnchorProps = {
  anchor: "right" | "left" | "bottom" | "top";
};

const Dragger = styled(
  "div",
  {}
)<AnchorProps>(({ theme, anchor }) => ({
  position: "absolute",

  ...(["left", "right"].includes(anchor) && {
    cursor: "ew-resize",
    width: "5px",
    // padding: "4px 0 0",
    // borderTop: "1px solid #ddd",

    top: 0,
    bottom: 0,
    right: anchor === "left" || anchor === undefined ? 0 : undefined,
    left: anchor === "right" ? 0 : undefined,
  }),
  ...(["top", "bottom"].includes(anchor) && {
    cursor: "ns-resize",
    height: "5px",

    // padding: "4px 0 0",
    // borderTop: "1px solid #ddd",
    top: anchor === "bottom" ? 0 : undefined,
    bottom: anchor === "top" ? 0 : undefined,
    right: 0,
    left: 0,
  }),

  zIndex: 100,
  backgroundColor: "#f4f7f9",
  // backgroundColor: "red",
}));

const MIN_DRAWER_WIDTH = 200;
const MAX_DRAWER_WIDTH = 800;
const MIN_DRAWER_HEIGHT = 50;
const MAX_DRAWER_HEIGHT = 600;

// if (["right", "left"].includes(anchor)) {
//   width = props.width || MIN_DRAWER_WIDTH;
// } else if (["top", "bottom"].includes(anchor)) {
//   height = props.height || MIN_DRAWER_HEIGHT;
// }

type SizeProps = {
  size?: { width?: number; height?: number };
};

const RootBox = styled(
  _Box,
  {}
)<AnchorProps & SizeProps>(({ theme, anchor, size }) => ({
  position: "absolute",
  zIndex: 100,

  ...(["left", "right"].includes(anchor) && {
    top: 0,
    bottom: 0,
    left: anchor === "left" ? 0 : undefined,
    right: anchor === "left" ? undefined : 0,

    ...(size && {
      width: size.width || MIN_DRAWER_WIDTH,
    }),
  }),
  ...(["top", "bottom"].includes(anchor) && {
    top: anchor === "top" ? 0 : undefined,
    bottom: anchor === "top" ? undefined : 0,
    left: 0,
    right: 0,
    ...(size && {
      height: size.height || MIN_DRAWER_HEIGHT,
    }),
  }),
}));

const Box = styled(
  _Box,
  {}
)<AnchorProps>(({ theme, anchor }) => ({
  position: "absolute",
  top: 0,
  bottom: 0,
  left: 0,
  right: 0,

  ...(["right"].includes(anchor) && {
    left: 5,
  }),
  ...(["bottom"].includes(anchor) && {
    top: 5,
  }),
}));

export type ResizableBoxProps = {
  width?: number | undefined;
  height?: number | undefined;
  anchor?: "right" | "left" | "bottom" | "top" | undefined;
  onResizeWidth?: (width: number) => void;
  onResizeHeight?: (height: number) => void;
  onResizeComplete?: () => void;
} & BoxProps;

export default function ResizableBox(props: ResizableBoxProps) {
  const {
    onResizeWidth,
    onResizeHeight,
    onResizeComplete,
    children,
    sx,
    ...other
  } = props;

  const boxRef = useRef<HTMLDivElement>(null);
  const anchor = props.anchor || "left";

  const handleMouseDown = (
    event: React.MouseEvent<HTMLDivElement, MouseEvent>
  ) => {
    event.stopPropagation();

    document.addEventListener("mouseup", handleMouseUp, false);
    document.addEventListener("mousemove", handleMouseMove, false);
  };

  const handleMouseUp = (event: MouseEvent) => {
    event.preventDefault();

    document.removeEventListener("mouseup", handleMouseUp, false);
    document.removeEventListener("mousemove", handleMouseMove, false);

    if (onResizeComplete) onResizeComplete();
  };

  const handleMouseMove = useCallback(
    (event: MouseEvent) => {
      event.preventDefault();

      if (boxRef && boxRef.current) {
        if (["right", "left"].includes(anchor)) {
          const rect = boxRef.current.getBoundingClientRect();
          let newWidth = MIN_DRAWER_WIDTH;

          if (anchor === "left") {
            newWidth = event.clientX - rect.x;
          } else {
            newWidth = rect.x + rect.width - event.clientX;
          }

          newWidth = Math.min(
            Math.max(newWidth, MIN_DRAWER_WIDTH),
            MAX_DRAWER_WIDTH
          );

          if (onResizeWidth) onResizeWidth(newWidth);
        } else {
          const rect = boxRef.current.getBoundingClientRect();
          let newHeight = MIN_DRAWER_HEIGHT;

          if (anchor === "top") {
            newHeight = event.clientY - rect.top;
          } else {
            newHeight = rect.bottom - event.clientY;
          }

          newHeight = Math.min(
            Math.max(newHeight, MIN_DRAWER_HEIGHT),
            MAX_DRAWER_HEIGHT
          );
          if (onResizeHeight) onResizeHeight(newHeight);
        }
      }
    },
    [anchor, boxRef, onResizeHeight, onResizeWidth]
  );
  
  return (
    <RootBox
      ref={boxRef}
      anchor={anchor}
      sx={sx}
      size={{ width: props.width, height: props.height }}
      {...other}
    >
      <Dragger onMouseDown={(e) => handleMouseDown(e)} anchor={anchor} />
      <Box anchor={anchor}>{children}</Box>
    </RootBox>
  );
}