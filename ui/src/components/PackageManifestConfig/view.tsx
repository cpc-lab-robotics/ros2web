import { styled } from "@mui/material/styles";
import Typography from "@mui/material/Typography";
import Grid from "@mui/material/Grid";
import Stack from "@mui/material/Stack";

import { PackageManifest } from "../../models/package";
import { Box } from "@mui/material";

const LabelTypo = styled(Typography)(({ theme }) => ({
  color: theme.palette.primary.main,
  fontWeight: "bold",
}));

type Props = {
  packageManifest: PackageManifest;
};

export default function PackageManifestView(props: Props) {
  const { packageManifest:pkg } = props;

  return (
    <Box sx={{ p: 2 }}>
      <Grid container spacing={1}>
        <Grid item xs={12}>
          <Grid container spacing={0}>
            <Grid item xs={12}>
              <LabelTypo variant="overline">Package Name</LabelTypo>
            </Grid>

            <Grid item xs={12}>
              <Typography variant="h5">{pkg.name}</Typography>
            </Grid>
          </Grid>
        </Grid>

        <Grid item xs={12}>
          <Grid container spacing={0}>
            <Grid item xs={12}>
              <LabelTypo variant="overline">Description</LabelTypo>
            </Grid>

            <Grid item xs={12}>
              <Typography variant="body1" color="text.secondary">
                {pkg.description}
              </Typography>
            </Grid>
          </Grid>
        </Grid>

        <Grid item xs={12}>
          <Grid container spacing={0}>
            <Grid item xs={12}>
              <LabelTypo variant="overline">version</LabelTypo>
            </Grid>

            <Grid item xs={12}>
              <Typography variant="body1" color="text.secondary">
                {pkg.version}
              </Typography>
            </Grid>
          </Grid>
        </Grid>

        <Grid item xs={12}>
          <Grid container spacing={0}>
            <Grid item xs={12}>
              <LabelTypo variant="overline">license</LabelTypo>
            </Grid>
            <Grid item xs={12}>
              {pkg.licenses.map((license) => (
                <Typography variant="body1" color="text.secondary">
                  {license}
                </Typography>
              ))}
            </Grid>
          </Grid>
        </Grid>

        <Grid item xs={12}>
          <Grid container spacing={0}>
            <Grid item xs={12}>
              <LabelTypo variant="overline">Maintainer</LabelTypo>
            </Grid>

            <Grid item xs={12}>
              {pkg.maintainers.map((parson) => (
                <Stack direction="row" spacing={3}>
                  <Typography variant="body1" color="text.secondary">
                    {parson.name}
                  </Typography>
                  <Typography variant="body1" color="text.secondary">
                    {parson.email}
                  </Typography>
                </Stack>
              ))}
            </Grid>
          </Grid>
        </Grid>

        <Grid item xs={12}>
          <Grid container spacing={0}>
            <Grid item xs={12}>
              <LabelTypo variant="overline">Auther</LabelTypo>
            </Grid>

            <Grid item xs={12}>
              {pkg.authors &&
                pkg.authors.map((parson) => (
                  <Stack direction="row" spacing={3}>
                    <Typography variant="body1" color="text.secondary">
                      {parson.name}
                    </Typography>
                    <Typography variant="body1" color="text.secondary">
                      {parson.email}
                    </Typography>
                  </Stack>
                ))}
            </Grid>
          </Grid>
        </Grid>

        <Grid item xs={12}></Grid>
      </Grid>
    </Box>
  );
}
