import React from "react";
import ReactDOM from "react-dom/client";


import { QueryClient, QueryClientProvider } from "react-query";
import { ReactQueryDevtools } from "react-query/devtools";

import { ThemeProvider } from "@mui/material/styles";
import CssBaseline from "@mui/material/CssBaseline";

import { theme } from "@/styles/theme";
import Routes from "./routes";

const queryClient = new QueryClient({
  defaultOptions: {
    queries: {
      staleTime: Infinity,
    },
  },
});

function App() {
  return (
    <QueryClientProvider client={queryClient}>
      <ThemeProvider theme={theme}>
        <CssBaseline />
        <Routes />
      </ThemeProvider>
    </QueryClientProvider>
  );
}

// function App() {
//   return (
//     <React.StrictMode>
//       <QueryClientProvider client={queryClient}>
//         <ThemeProvider theme={theme}>
//           <CssBaseline />
//           <Routes />
//         </ThemeProvider>
//         <ReactQueryDevtools initialIsOpen />
//       </QueryClientProvider>
//     </React.StrictMode>
//   );
// }

ReactDOM.createRoot(document.getElementById("root")!).render(<App />);
