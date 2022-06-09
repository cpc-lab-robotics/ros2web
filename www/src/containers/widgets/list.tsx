import React from "react";

import { useProps } from "@/services/api";
import { Data } from "@/models";

import List, { ListProps } from "@/components/List";
import { ItemData } from "@/components/List/types";

import { WidgetProps, WidgetEvent } from "./types";

export type ListWidgetProps = {
  on_selected?: (event: WidgetEvent) => void;
  selected_id?: string;
};
type Props = ListWidgetProps & WidgetProps;

export default function ListWidget(props: Props) {
  const {
    webPackageName,
    bindStateKey,
    initState,
    setState,
    dataAdaptor,
    on_selected,
    selected_id,
  } = props;
  
  let items: ItemData[] = [];
  if (dataAdaptor) {
    items = dataAdaptor.useItems();
  }
  
  let updateProps: Record<string, any> = {}
  if (webPackageName && initState && bindStateKey) {
    const bindProps: Record<string, string> = {};
    for (const prop of ["selected_id"]){
      const stateKey = bindStateKey[prop];
      if(stateKey) {
        bindProps[stateKey] = prop;
      }
    }
    updateProps = useProps(
      webPackageName,
      initState,
      bindProps
    );
  }

  const selectedHandler = (item?: ItemData) => {
    // let data: Data | undefined = undefined;
    // if (dataAdaptor && item) {
    //   data = dataAdaptor.itemToData(item);
    // }
    if (on_selected) {
      const event: WidgetEvent = {
        event: {
          widget_id: "__id__",
          type: "on_selected",
        },
        value: (item) ? item.id : null,
      };
      on_selected(event);
    }
  };

  const listProps: ListProps = {
    items,
    onSelected: selectedHandler,
    selectedId: updateProps["selected_id"] || selected_id,
  };
  
  return <List {...listProps} />;
}
