import React from "react";
import Button from "@@mui/material/Button";
import Stack from "@@mui/material/Stack";

export default function Widget(props: any) {
  const { on_click, label } = props;

  const clickHandler = async () => {
    if (on_click){
       on_click();
    }else{
      await fetch("/api/extension/@plugin_name/click");
    }
  };

  const buttonLabel = label || "Hello";

  return (
    <Stack sx={{ p: 1 }}>
      <Button onClick={clickHandler} variant="outlined">
        {buttonLabel}
      </Button>
    </Stack>
  );
}
