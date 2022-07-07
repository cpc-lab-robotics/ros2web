import TableCell from "@mui/material/TableCell";
import TableRow from "@mui/material/TableRow";
import Typography from "@mui/material/Typography";

export function ErrorTableRow(key: string, name: string, message: string) {
  return (
    <TableRow key={key}>
      <TableCell
        component="th"
        scope="row"
        style={{
          whiteSpace: "nowrap",
          overflow: "hidden",
          textOverflow: "ellipsis",
          // maxWidth: 100,
        }}
      >
        <Typography variant="caption" sx={{ color: "#555" }}>
          {name}
        </Typography>
        {": "}
        <Typography variant="overline" sx={{ color: "#555" }}>
          {message}
        </Typography>
      </TableCell>
    </TableRow>
  );
}
