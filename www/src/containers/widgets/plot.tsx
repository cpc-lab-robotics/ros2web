import Plot, { PlotProps } from "@/components/Plot";
import Stack from "@mui/material/Stack";

import { WidgetProps, WidgetEvent } from "./types";

type PlotWidgetProps = {
  data?: any[];
  layout?: Record<string, any>;
  config?: any;
  style?: any;
} & WidgetProps;

export default function PlotWidget(_props: PlotWidgetProps) {
  const { dataAdaptor, ...rest } = _props;

  let props: PlotWidgetProps = {
    data: [],
    layout: {},
    config: undefined,
    style: undefined,
    ...rest,
  };

  let _data: any[] | undefined = undefined;

  if (dataAdaptor) {
    props = dataAdaptor.use<PlotWidgetProps>(props);
    _data = dataAdaptor.getProps();
  }
  const { data, layout, config, style } = props;

  const plotProps: PlotProps = {
    data: _data || data,
    layout,
    config,
    style,
  };
  return (
    <Stack justifyContent="center" alignItems="center">
      <Plot {...plotProps} />
    </Stack>
  );
}
