import React from "react";
import ReactDOM from "react-dom/client";
import { QueryClient, QueryClientProvider } from "react-query";
import { ReactQueryDevtools } from "react-query/devtools";

import { styled } from "@mui/material/styles";
import Grid from "@mui/material/Grid";
import Paper from "@mui/material/Paper";
import Box from "@mui/material/Box";

import Config from "@/containers/Config";

const queryClient = new QueryClient({
  defaultOptions: {
    queries: {
      staleTime: Infinity,
    },
  },
});

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

const Page = () => (
  <Box sx={{ flexGrow: 1 }}>
    <Grid container spacing={0}>
      <Grid item xs={4}>
        <Item height={300} elevation={0}>
          <Config
            node_name={"/turtlesim"}
            param_names={["background_b", "background_g", "background_r"]}
          />
        </Item>
      </Grid>
    </Grid>
  </Box>
);

function App() {
  return (
    <React.StrictMode>
      <QueryClientProvider client={queryClient}>
        <Page />
        <ReactQueryDevtools initialIsOpen />
      </QueryClientProvider>
    </React.StrictMode>
  );
}

ReactDOM.createRoot(document.getElementById("root")!).render(<App />);
