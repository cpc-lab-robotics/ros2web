import { useTheme, styled } from "@mui/material/styles";

import Typography from "@mui/material/Typography";
import Box from "@mui/material/Box";

export default function Blank(props: any) {
  const theme = useTheme();
  
  return (
    <Box sx={{ m:-1, bgcolor: theme.palette.secondary.main, width:"110%", height:"110%" }}>
      <Box sx={{p:2}}><Typography variant="overline" color={"#555"}>Disable</Typography></Box>
    </Box>
  );
}