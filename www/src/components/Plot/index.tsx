import React from "react";

import _Plot, { PlotParams } from "react-plotly.js";

// export type Point = {
//   dataIndex: number;
//   id: number;
//   x?: number;
//   y?: number;
//   z?: number;
// };

export interface PlotProps {
  data?: any[];
  layout?: Record<string, any>;
  config?: any;
  // merge?: boolean;
  style?: any;
}

type PlotState = {
  data: any[];
  layout: Record<string, any>;
  frames?: any;
  config?: any;
};

class Plot extends React.Component<PlotProps, PlotState> {
  constructor(props: PlotProps) {
    super(props);
    const { data, layout, config } = props;
    this.state = {
      data: data ? data : [],
      layout: layout ? layout : {},
      config,
    };
  }
  
  static getDerivedStateFromProps(nextProps: PlotProps, prevState: PlotState) {
    const { data, layout } = nextProps;
    // const { data: prevData } = prevState;

    return { data, layout };
  }
  
  // componentDidMount() {
  //   console.log("componentDidMount");
  // }
  // shouldComponentUpdate(nextProps: PlotProps, nextState: PlotState) {
  //   return true;
  // }
  // componentDidUpdate(prevProps: PlotProps, prevState: PlotState) {
  //   console.log("componentDidUpdate");
  // }
  // componentWillUnmount() {
  //   console.log("componentWillUnmount");
  // }

  render() {
    const { style } = this.props;
    const props = {
      ...(style ? {style} : {})
    }
    // const handlers: Record<string, any> = {};
    // Object.entries(this.props).forEach(([key, value]) => {
    //   let match = key.match(/^on/);

    //   if (match && !["onInitialized", "onUpdate"].includes(key)) {
    //     handlers[key] = value;
    //   }
    // });
    return (
      <_Plot
        useResizeHandler
        data={this.state.data}
        layout={this.state.layout}
        frames={this.state.frames}
        config={this.state.config}
        onInitialized={(figure) => this.setState(figure)}
        onUpdate={(figure) => this.setState(figure)}
        {...props}
      />
    );
  }
}

export default Plot;
