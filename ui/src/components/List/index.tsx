import _List from "@mui/material/List";
import ListItemButton from "@mui/material/ListItemButton";
import ListItemText from "@mui/material/ListItemText";

import { ItemData } from "./types"

export type ListProps = {
  items: ItemData[];
  onSelected: (item?: ItemData) => void;
  selectedId?: string;
};

export default function List(props: ListProps) {
  const { items, onSelected, selectedId } = props;
  const handleListItemClick = (item: ItemData) => {
    const renew: boolean = selectedId !== item.id;
    if (onSelected && renew) onSelected(item);
  };

  return (
    <_List
      sx={{
        padding: 0,
        height: "100%",
        overflowY: "auto",
      }}
    >
      {items &&
        items.map((item) => {
          return (
            <ListItemButton
              key={item.id}
              onClick={() => handleListItemClick(item)}
              selected={(selectedId) ? selectedId === item.id : false}
            >
              <ListItemText primary={item.primary} secondary={item.secondary} />
            </ListItemButton>
          );
        })}
    </_List>
  );
}
