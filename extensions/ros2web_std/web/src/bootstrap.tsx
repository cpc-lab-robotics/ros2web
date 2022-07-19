import React from "react";
import ReactDOM from "react-dom/client";

import { styled } from "@mui/material/styles";
import Grid from "@mui/material/Grid";
import Paper from "@mui/material/Paper";
import Box from "@mui/material/Box";

import Button from "@/components/Button";
import List, { ItemData } from "@/components/List";
import ButtonGroup from "@/components/ButtonGroup";

const items: ItemData[] = Array(10)
  .fill(0)
  .map((_, index) => ({
    id: `index-${index}`,
    primary: `primary-${index}`,
    secondary: `secondary-${index}`,
  }));

const BaseBox = styled(Box)(({ theme }) => ({
  backgroundColor: "#eee",
  border: "1px solid #ccc",
  "*": {
    boxSizing: "border-box",
  },
}));

const Item = (props: any) => {
  const { children, height, elevation } = props;
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

const App = () => (
  <Box sx={{ flexGrow: 1 }}>
    <Grid container spacing={0}>
      <Grid item xs={4}>
        <Item height={300} elevation={0}>
          <Button
            justify_content={"center"}
            align_items={"center"}
            labels={["label1", "label2"]}
          />
        </Item>
      </Grid>
      <Grid item xs={4}>
        <Item height={300} elevation={1}>
          <List items={items} />
        </Item>
      </Grid>
      <Grid item xs={4}>
        <Item height={300} elevation={1}>
          <ButtonGroup
            variant="text"
            disable_elevation={true}
            orientation="vertical"
            color="secondary"
            size="small"
            labels={["label1", "label2", "label2"]}
          />
        </Item>
      </Grid>
    </Grid>
  </Box>
);

ReactDOM.createRoot(document.getElementById("root")!).render(<App />);
