import React from 'react';
import ReactDOM from "react-dom/client";

import { styled } from "@@mui/material/styles";
import Grid from "@@mui/material/Grid";
import Paper from "@@mui/material/Paper";
import Box from "@@mui/material/Box";

import Widget from './Widget';

const elevation = 0;

const App = () => (
  <Box sx={{ flexGrow: 1 }}>
    <Grid container spacing={0}>
      <Grid item xs={4}>
        <Item height={300}>
          <Widget />
        </Item>
      </Grid>
    </Grid>
  </Box>
);

ReactDOM.createRoot(document.getElementById("root")!).render(<App />);


const BaseBox = styled(Box)(({ theme }) => ({
  backgroundColor: "#ccc",
  "*": {
    boxSizing: "border-box",
  },
}));

const Item = (props: any) => {
  const { children, height } = props;
  return (
    <BaseBox
      sx={{
        height,
        position: "relative",
        width: "100%",
      }}
    >
      <Box
        sx={{
          position: "absolute",
          top: 0,
          bottom: 0,
          left: 0,
          right: 0,
          p: 1,
        }}
      >
        <Paper
          elevation={elevation}
          sx={{
            bgcolor: elevation ? undefined : "rgba(0,0,0,0)",
            position: "relative",
            p: elevation ? 1 : 0,
            width: "100%",
            height: "100%",
            overflow: "hidden",
            border: elevation ? "1px solid #ddd" : "none",
          }}
        >
          {children}
        </Paper>
      </Box>
    </BaseBox>
  );
};


