import { styled } from "@mui/material/styles";

const Main = styled("main", {
  shouldForwardProp: (prop) =>
    prop !== "leftWidth" &&
    prop !== "leftOpen" &&
    prop !== "rightWidth" &&
    prop !== "rightOpen",
})<{
  leftWidth: number;
  leftOpen?: boolean;
  rightWidth: number;
  rightOpen?: boolean;
}>(({ theme, leftWidth, leftOpen, rightWidth, rightOpen }) => ({
  flexGrow: 1,
  overflow: "hidden",
  transition: theme.transitions.create("margin", {
    easing: theme.transitions.easing.sharp,
    duration: theme.transitions.duration.leavingScreen,
  }),

  marginLeft: `-${leftWidth}px`,
  ...(leftOpen && {
    transition: theme.transitions.create("margin", {
      easing: theme.transitions.easing.easeOut,
      duration: theme.transitions.duration.enteringScreen,
    }),
    marginLeft: 0,
  }),

  marginRight: `-${rightWidth}px`,
  ...(rightOpen && {
    transition: theme.transitions.create('margin', {
      easing: theme.transitions.easing.easeOut,
      duration: theme.transitions.duration.enteringScreen,
    }),
    marginRight: 0,
  }),
}));

export default Main;
