import TableCell from "@mui/material/TableCell";
import TableRow from "@mui/material/TableRow";
import Slider from "@mui/material/Slider";
import TextField from "@mui/material/TextField";

type Range = {
  fromValue: number;
  toValue: number;
  step?: number;
} 

export function SliderTableRow(
  key: string,
  name: string,
  value: number,
  range: Range,
  onChange: (value:any) => void,
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
      
      <TableCell align="right" style={{ minWidth: 100 }}>
        <Slider
          defaultValue={value}
          step={range.step}
          min={range.fromValue}
          max={range.toValue}
          value={value}
          onChange={(event, value)=>onChange(value)}
          onChangeCommitted={onChangeCommitted}
        />
      </TableCell>

      <TableCell align="right" style={{ width: 100 }}>
        <TextField
          size="small"
          inputProps={{
            sx: {
              textAlign: "right",
            },
          }}
          value={value}
          variant="outlined"
          onChange={(event)=>{
            const v = Number(event.target.value);
            if(!isNaN(v)){
              if(range.fromValue <= v && v <= range.toValue)
                onChange(v)
            }
          }}
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