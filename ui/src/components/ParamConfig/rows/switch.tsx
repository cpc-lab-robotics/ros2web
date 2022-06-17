import TableCell from "@mui/material/TableCell";
import TableRow from "@mui/material/TableRow";
import Switch from "@mui/material/Switch";

export function SwitchTableRow(
  key: string,
  name: string,
  value: boolean,
  onChange: (value: any) => void,
) {
  return (
    <TableRow key={key}>
      <TableCell
        component="th"
        scope="row"
        style={{
          whiteSpace: "nowrap",
          overflow: "hidden",
          textOverflow: "ellipsis",
          maxWidth: 100,
        }}
      >
        {name}
      </TableCell>
      <TableCell colSpan={2} align="center">
        <Switch
          checked={value}
          onChange={(event) => onChange(event.target.value)}
        />
      </TableCell>
    </TableRow>
  );
}
