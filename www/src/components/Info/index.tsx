import React from "react";

import { styled } from "@mui/material/styles";
import Typography from "@mui/material/Typography";
import Box from "@mui/material/Box";
import Stack from "@mui/material/Stack";
import List from "@mui/material/List";
import ListSubheader from "@mui/material/ListSubheader";
import Divider from '@mui/material/Divider';


import { InfoData, KeyValueWithHeader, KeyValue } from "./types";

const ItemHeader = styled(ListSubheader)(({ theme }) => ({
  ...theme.typography.body1,
  color: theme.palette.primary.main,
  fontWeight: "bold",
  // backgroundColor: "#ddd",
  // borderLeft: `10px solid ${theme.palette.secondary.main}`
}));

export const LabelTypo = styled(Typography)(({ theme }) => ({
  color: theme.palette.primary.main,
  fontWeight: "bold",
}));

type KeyValueWithHeaderProps = {
  item: KeyValueWithHeader;
};
function InfoKeyValueWithHeader(props: KeyValueWithHeaderProps) {
  const { item } = props;

  return (
    <React.Fragment>
      <ItemHeader>{item.header}</ItemHeader>

      {item.keyValues.map((item, index) => (
        <InfoKeyValue key={index} item={item} />
      ))}
    </React.Fragment>
  );
}

type KeyValueProps = {
  item: KeyValue;
};

function InfoKeyValue(props: KeyValueProps) {
  const { item } = props;

  return (
    <Stack
      direction="row"
      justifyContent="space-between"
      // divider={<Divider orientation="vertical" flexItem />}
      spacing={1}
    >
      <LabelTypo> {item.key}</LabelTypo>
      <Box> {String(item.value)}</Box>
    </Stack>
  );
}

export type InfoProps = {
  info?: InfoData;
};

function isKeyValueWithHeader(item: any): item is KeyValueWithHeader {
  return item.hasOwnProperty("header");
}

export default function Info(props: InfoProps) {
  const { info } = props;

  if (info === undefined || info === null) {
    return (
      <Box sx={{ p: 3, width: "100%", height: "100%" }}>
        <Stack
          sx={{ height: "100%" }}
          justifyContent="center"
          alignItems="center"
        >
          <Typography variant="overline" color="text.secondary">
            Not data.
          </Typography>
        </Stack>
      </Box>
    );
  }

  return (
    <List
      sx={{
        p: 1,
        height: "100%",
        overflowY: "auto",
      }}
      subheader={<ListSubheader component="div">{info.title}</ListSubheader>}
    >
      {info.items.map((item, index) => {
        if (isKeyValueWithHeader(item)) {
          return <InfoKeyValueWithHeader key={index} item={item} />;
        } else {
          return <InfoKeyValue key={index} item={item} />;
        }
      })}
    </List>
  );
}
