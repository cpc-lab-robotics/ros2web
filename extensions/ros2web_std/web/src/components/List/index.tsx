import _List from "@mui/material/List";
import ListItemButton from "@mui/material/ListItemButton";
import ListItemText from "@mui/material/ListItemText";

export type ItemData = {
  id: string;
  primary: string;
  secondary?: string;
};

export type ListProps = {
  items: ItemData[];
  on_selected?: (item?: ItemData) => void;
  selected_id?: string;
};

export default function List(props: ListProps) {
  const { items, on_selected, selected_id } = props;

  const handleListItemClick = (item: ItemData) => {
    const renew: boolean = selected_id !== item.id;
    if (on_selected && renew) on_selected(item);
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
              selected={(selected_id) ? selected_id === item.id : false}
            >
              <ListItemText primary={item.primary} secondary={item.secondary} />
            </ListItemButton>
          );
        })}
    </_List>
  );
}
