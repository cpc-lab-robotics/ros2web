import { ThemeProvider } from "@mui/material/styles";
import { theme } from "../src/styles/theme";
import { QueryClient, QueryClientProvider } from "react-query";
import { initialize, mswDecorator } from 'msw-storybook-addon';

// Initialize MSW
initialize();

const queryClient = new QueryClient({
  defaultOptions: {
    queries: {
      staleTime: Infinity,
    },
  },
});

export const decorators = [
  mswDecorator,
  (Story) => (
    <QueryClientProvider client={queryClient}>
      <ThemeProvider theme={theme}>
        <Story />
      </ThemeProvider>
    </QueryClientProvider>
  ),
];

export const parameters = {
  actions: { argTypesRegex: "^on[A-Z].*" },
  controls: {
    matchers: {
      color: /(background|color)$/i,
      date: /Date$/,
    },
  },
};
