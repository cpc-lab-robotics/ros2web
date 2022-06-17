import * as React from "react";

import { styled } from "@mui/material/styles";
import _Tabs from "@mui/material/Tabs";
import _Tab from "@mui/material/Tab";
import Box, {BoxProps} from "@mui/material/Box";

export const Tabs = styled(_Tabs)(({ theme }) => ({
  // ".MuiTabs-indicator": {
  //   backgroundColor: "transparent",
  // }
  borderBottom: "1px solid #DDDDDD",
}));

export const Tab = styled(_Tab)(({ theme }) => ({
  zIndex: 100,
  // borderRight: theme.border.thin,
  // minWidth: 72,
  // minHeight: 32,
  // padding: theme.spacing(1),
  // fontFamily: [
  //   "-apple-system",
  //   "BlinkMacSystemFont",
  //   "Segoe UI",
  //   "Roboto",
  //   "Helvetica Neue",
  //   "Arial",
  //   "sans-serif",
  //   "Apple Color Emoji",
  //   "Segoe UI Emoji",
  //   "Segoe UI Symbol",
  // ].join(","),
  // color: 'black',
  // fontSize: theme.typography.pxToRem(12),
  // "&.Mui-selected": {
  //   color: "#FFF",
  //   backgroundColor: theme.palette.secondary.main,
  // },
  // "&:hover": {},
  // "&:focus": {},
}));

type TabPanelProps<T> = {
  children?: React.ReactNode;
  tabValue: T;
  value: T;
} & BoxProps;

export default function TabPanel<T>(props: TabPanelProps<T>) {
  const { children, value, tabValue, ...other } = props;

  return (
    <Box
      role="tabpanel"
      hidden={value !== tabValue}
      {...other}
    >
      {children}
    </Box>
  );
}