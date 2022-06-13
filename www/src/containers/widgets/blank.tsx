import { WidgetProps } from "./types";

type BlankWidgetProps = {
} & WidgetProps;

export default function BlankWidget(props: BlankWidgetProps) {
  const { dataAdaptor } = props;
  if (dataAdaptor) {
    // tableProps = dataAdaptor.use<BlankWidgetProps>({});
    
    return (<></>);
  }else{
    return (<></>);
  }
}
