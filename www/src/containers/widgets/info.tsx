import Info, { InfoProps } from "@/components/Info";
import { useProps } from "@/services/api";
import { Data } from "@/models";

import { WidgetProps } from "./types";
import { InfoData } from "@/components/Info/types";

export type ListWidgetProps = {
  data: any;
};
type Props = ListWidgetProps & WidgetProps;

export default function InfoWidget(props: Props) {
  const {
    webPackageName,
    bindStateKey,
    initState,
    setState,
    dataAdaptor,
    data,
    ...rest
  } = props;
  
  let infoData: InfoData | undefined = undefined;
  if (dataAdaptor) {
    infoData = dataAdaptor.useInfo();
  }

  // let updateProps: Record<string, any> = {}
  // if (webPackageName && initState && bindStateKey) {
  //   const bindProps: Record<string, string> = {};
  //   for (const prop of ["data"]){
  //     const stateKey = bindStateKey[prop];
  //     if(stateKey) {
  //       bindProps[stateKey] = prop;
  //     }
  //   }
  //   updateProps = useProps(
  //     webPackageName,
  //     initState,
  //     bindProps
  //   );
  // }

  const infoProps: InfoProps = {
    info:infoData
  }
  return <Info {...infoProps}/>;
}