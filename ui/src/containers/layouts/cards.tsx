import React, { useState, useEffect } from "react";
import { useProps } from "@/services/api";
import { useWindowSize, useGlobalState } from "@/hooks";

import Masonry from "@mui/lab/Masonry";
import Box from "@mui/material/Box";
import Card from "@mui/material/Card";
import CardContent from "@mui/material/CardContent";
import CardHeader from "@mui/material/CardHeader";

import Typography from "@mui/material/Typography";

import { LayoutProps } from "./types";

export type CardLayoutProps = {};
type Props = CardLayoutProps & LayoutProps;

export default function CardLayout(props: Props) {
  const {
    children,
    webPackageName,
    initState,
    bindStateKey,
    setState,
    ...rest
  } = props;

  const windowSize = useWindowSize();
  const [resizeDate, setResizeDate] = useGlobalState<Date>(
    ["resizeEvent"],
    new Date()
  );
  useEffect(() => {
    setResizeDate(new Date());
  }, [windowSize]);

  let updateProps: Record<string, any> = {};
  if (webPackageName && initState && bindStateKey) {
    const bindProps: Record<string, string> = {};
    // for (const prop in ["name"]){
    //   const stateKey = bindStateKey[prop];
    //   if(stateKey) {
    //     bindProps[stateKey] = prop;
    //   }
    // }
    // updateProps = useProps(
    //   webPackageName,
    //   initState,
    //   bindProps
    // );
  }
  const cards: React.ReactElement[] = [];
  const childElements = React.Children.toArray(children);

  let count = 0;
  for (const childElement of childElements) {
    if (React.isValidElement(childElement)) {
      const { card_name } = childElement.props;

      cards.push(
        <Card key={childElement.key}>
          <CardHeader
            title={card_name}
            titleTypographyProps={{
              variant: "overline",
              color: "text.primary",
            }}
            sx={{
              borderBottom: "1px solid #ccc",
            }}
          />
          <CardContent sx={{ p: 0, '&:last-child': { p: 0 }}}>{childElement}</CardContent>
        </Card>
      );
    }
  }

  return (
    <Box sx={{ p: 1 }}>
      <Masonry
        sx={{ m: 0, p: 0 }}
        spacing={1}
        columns={{ xs: 1, sm: 2, md: 3, lg: 4, xl: 5 }}
      >
        {cards}
      </Masonry>
    </Box>
  );
}
