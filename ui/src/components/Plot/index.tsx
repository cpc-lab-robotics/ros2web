import React from "react";

import Plotly, { PlotParams } from "react-plotly.js";

export type PlotProps = {
  data?: any[];
  layout?: Record<string, any>;
  config?: any;
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
    return (
      <Plotly
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
