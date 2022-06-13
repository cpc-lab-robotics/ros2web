import TableCell from "@mui/material/TableCell";
import TableRow from "@mui/material/TableRow";
import TextField from "@mui/material/TextField";

export function TextTableRow(
  key: string,
  name: string,
  value: any,
  type: string,
  onChange: (value: any) => void,
  onChangeCommitted: () => void
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
      <TableCell colSpan={2} align="right">
        <TextField
          size="small"
          inputProps={{
            sx: {
              ...(type === "number" ? { textAlign: "right" } : {}),
            },
          }}
          variant="outlined"
          fullWidth={true}
          value={value}
          type={type}
          onChange={(event) => onChange(event.target.value)}
          onKeyDown={(event: any) => {
            if (event.keyCode === 13) {
              if (onChangeCommitted) onChangeCommitted();
            }
          }}
        />
      </TableCell>
    </TableRow>
  );
}
