import React from "react";

import { styled } from "@mui/material/styles";
import AppBar from "@mui/material/AppBar";
import Switch from "@mui/material/Switch";
import Toolbar from "@mui/material/Toolbar";
import Typography from "@mui/material/Typography";
import Paper from "@mui/material/Paper";
import Box from "@mui/material/Box";
import Grid from "@mui/material/Grid";
import Stack from "@mui/material/Stack";

import _GridLayout from "react-grid-layout/";
import { Responsive, WidthProvider } from "react-grid-layout";
const ResponsiveReactGridLayout = WidthProvider(Responsive);

import "react-grid-layout/css/styles.css";
import "react-resizable/css/styles.css";
import "./styles.scss";

const Div = styled("div")(({ theme }) => ({
  width: "100%",
  paddingTop: `${theme.padding.dense + 1}px`,
}));

export const breakpoints = ["lg", "md", "sm", "xs", "xxs"] as const;
export type BreakpointType = typeof breakpoints[number];
export function isBreakpoint(breakpoint: string): breakpoint is BreakpointType {
  return breakpoints.includes(breakpoint as BreakpointType);
}

export type GridItem = {
  element: React.ReactElement;
  elevation?: number;
} & _GridLayout.Layout;

export type GridProps = {
  onLayoutChange?: (items: GridItem[]) => void;
  title?: string;
  rowHeight: number;
  cols: { lg: number; md: number; sm: number; xs: number; xxs: number };
  initialLayout: GridItem[];
  margin: [number, number];
};

type GridState = {
  currentBreakpoint: BreakpointType;
  compactType: "vertical" | "horizontal" | null;
  mounted: boolean;
  static: boolean;
  layout: GridItem[];
};

class GridLayout extends React.Component<GridProps, GridState> {
  public static defaultProps: GridProps = {
    rowHeight: 50,
    cols: { lg: 12, md: 8, sm: 6, xs: 4, xxs: 2 },
    initialLayout: [],
    margin: [0, 0],
  };

  constructor(props: GridProps) {
    super(props);

    this.state = {
      currentBreakpoint: "lg",
      compactType: null,
      mounted: false,
      static: true,
      layout: this.props.initialLayout,
    };
  }

  componentDidMount() {
    this.setState({ mounted: true });
  }

  generateDOM() {
    return this.state.layout.map((item, index) => {
      const { widgetName } = item.element.props;
      const elevation = item.elevation ?? 1;
      return (
        <Box
          key={item.element.key}
          className={this.state.static ? "static" : ""}
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
                position: "relative",
                p: elevation ? 1 : 0,
                width: "100%",
                height: "100%",
                overflow: "hidden",
                border: elevation ? "1px solid #ddd" : "none",
              }}
            >
              {item.element}
            </Paper>
          </Box>
          <Box
            sx={{
              position: "absolute",
              display: item.static ? "none" : "flex",
              userSelect: "none",
              bgcolor: "rgba(0, 0, 0, 0.5)",
              width: "100%",
              height: "100%",
              p: 1,
            }}
          >
            <Stack spacing={0}>
              <Typography variant="overline" sx={{ color: "#ddd" }}>
                {widgetName}
              </Typography>
              <Typography variant="caption" sx={{ color: "#ddd" }}>
                {`${item.x} × ${item.y}, w${item.w}, h${item.h}`}
              </Typography>
            </Stack>
          </Box>
        </Box>
      );
    });
  }

  onBreakpointChange = (newBreakpoint: string, newCols: number) => {
    if (isBreakpoint(newBreakpoint))
      this.setState({
        currentBreakpoint: newBreakpoint,
      });
  };

  onLayoutChange = (
    currentLayout: _GridLayout.Layout[],
    allLayouts: _GridLayout.Layouts
  ) => {
    const layout: GridItem[] = this.state.layout.map((item, index) => {
      const l = currentLayout[index] || {};
      return { ...item, ...l };
    });

    const { onLayoutChange } = this.props;
    if (onLayoutChange) onLayoutChange(layout);
    this.setState({ layout });
  };

  onChangeStatic = (event: React.ChangeEvent<HTMLInputElement>) => {
    const checked = event.target.checked;

    const layout: GridItem[] = this.state.layout.map((item, index) => {
      return { ...item, static: !checked };
    });

    this.setState({ layout, static: !checked });
  };

  render() {
    return (
      <Div>
        <AppBar color="inherit" position="fixed" elevation={1}>
          <Toolbar variant="dense">
            <Typography
              variant="overline"
              sx={{ userSelect: "none", flexGrow: 1 }}
            >
              {this.props.title}
            </Typography>
            <Switch onChange={this.onChangeStatic} />
          </Toolbar>
        </AppBar>

        <ResponsiveReactGridLayout
          {...this.props}
          layouts={{ lg: this.state.layout }}
          onBreakpointChange={this.onBreakpointChange}
          onLayoutChange={this.onLayoutChange}
          // WidthProvider option
          measureBeforeMount={true}
          // I like to have it animate on mount. If you don't, delete `useCSSTransforms` (it's default `true`)
          // and set `measureBeforeMount={true}`.
          // useCSSTransforms={this.state.mounted}
          useCSSTransforms={false}
          compactType={this.state.compactType}
          preventCollision={!this.state.compactType}
        >
          {this.generateDOM()}
        </ResponsiveReactGridLayout>
      </Div>
    );
  }
}

export default GridLayout;
