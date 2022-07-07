import React, { useEffect, Dispatch, SetStateAction } from "react";

import Main from "./Main";
import DrawerHeader from "./DrawerHeader";
import ResizeBox from "@/components/ResizeBox";

import { useTheme } from "@mui/material/styles";
import AppBar from "@mui/material/AppBar";
import Box, { BoxProps } from "@mui/material/Box";
import Drawer from "@mui/material/Drawer";
import CssBaseline from "@mui/material/CssBaseline";
import Toolbar from "@mui/material/Toolbar";
import Typography from "@mui/material/Typography";
import IconButton from "@mui/material/IconButton";
import MenuIcon from "@mui/icons-material/Menu";
import SearchIcon from "@mui/icons-material/Search";

export type ThreePaneProps = {
  name: string;
  leftSidebarWidth?: number;
  rightSidebarWidth?: number;
  disableLeftSidebar?: boolean;
  disableRightSidebar?: boolean;
  onChangeSize?: ()=>void;
} & BoxProps;

export default function ThreePane(props: ThreePaneProps) {
  const {
    name,
    leftSidebarWidth,
    rightSidebarWidth,
    disableLeftSidebar,
    disableRightSidebar,
    onChangeSize,
    children,
    sx,
    ...rest
  } = props;
  
  const initLeftBoxWidth = leftSidebarWidth || 240;
  const initRightBoxWidth = rightSidebarWidth || 240;
  const theme = useTheme();

  const [leftSidebarOpen, setLeftSidebarOpen] = React.useState(
    !disableLeftSidebar
  );
  const [rightSidebarOpen, setRightSidebarOpen] = React.useState(
    !disableRightSidebar
  );
  const [leftBoxWidth, setLeftBoxWidth] = React.useState(initLeftBoxWidth);
  const [rightBoxWidth, setRightBoxWidth] = React.useState(initRightBoxWidth);

  const childElements = React.Children.toArray(children);

  useEffect(() => {
    if (onChangeSize)
      onChangeSize();
  }, [leftBoxWidth, rightBoxWidth]);

  return (
    <Box
      sx={{
        flexGrow: 1,
        bgcolor: "background.paper",
        display: "flex",
        height: "100vh",
        // paddingTop: `${theme.padding.appbar}px`,
      }}
      {...rest}
    >

      <AppBar
        color="inherit"
        position="fixed"
        sx={{
          zIndex: theme.zIndex.drawer + 1,
        }}
        elevation={1}
      >
        <Toolbar variant="dense">
          <IconButton
            size="large"
            edge="start"
            color="inherit"
            aria-label="menu"
            sx={{ mr: 2, ...(disableLeftSidebar && { display: "none" }) }}
            onClick={() => {
              setLeftSidebarOpen(!leftSidebarOpen);
            }}
          >
            <MenuIcon />
          </IconButton>
          <Typography variant="button" component="div" sx={{ flexGrow: 1 }}>
            {name}
          </Typography>
          <IconButton
            color="inherit"
            sx={{ ...(disableRightSidebar && { display: "none" }) }}
            onClick={() => {
              setRightSidebarOpen(!rightSidebarOpen);
            }}
          >
            <SearchIcon />
          </IconButton>
        </Toolbar>
      </AppBar>

      <Drawer
        sx={{
          width: leftBoxWidth,
          flexShrink: 0,
          overflow: "hidden",
          "& .MuiDrawer-paper": {
            width: leftBoxWidth,
            boxSizing: "border-box",
            overflow: "hidden",
          },
          bgcolor: "rgba(0,0,0,0)",
          // ...(!leftSidebarOpen && { display: 'none' })
        }}
        variant="persistent"
        anchor="left"
        open={leftSidebarOpen}
      >
        <ResizeBox
          // sx={{ marginTop: theme.spacing(1) }}
          width={leftBoxWidth}
          onResizeWidth={(width) => setLeftBoxWidth(width)}
        >
          <DrawerHeader />
          {childElements[0] ? childElements[0] : <></>}
        </ResizeBox>
      </Drawer>

      <Main
        leftOpen={leftSidebarOpen}
        leftWidth={leftBoxWidth}
        rightOpen={rightSidebarOpen}
        rightWidth={rightBoxWidth}
        sx={{
          zIndex: theme.zIndex.drawer,
        }}
      >
        <DrawerHeader />
        {childElements[1] ? childElements[1] : <></>}
      </Main>

      <Drawer
        variant="persistent"
        anchor="right"
        open={rightSidebarOpen}
        sx={{
          width: rightBoxWidth,
          overflow: "hidden",
          flexShrink: 0,
          "& .MuiDrawer-paper": {
            width: rightBoxWidth,
            boxSizing: "border-box",
            overflow: "hidden",
          },
        }}
      >
        <ResizeBox
          width={rightBoxWidth}
          anchor="right"
          onResizeWidth={(width) => setRightBoxWidth(width)}
        >
          <DrawerHeader />
          {childElements[2] ? childElements[2] : <></>}
        </ResizeBox>
      </Drawer>
    </Box>
  );
}
