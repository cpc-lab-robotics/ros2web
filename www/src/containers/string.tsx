import { useQuery } from "react-query";
import { getStateValue } from "@/services/api";

export type StringProps = {
  webPackageName: string;
  stateKey: string;
  value: string;
};

export default function String(props: StringProps) {
  const { webPackageName, stateKey, value} = props;
  
  const { data } = useQuery(
    [webPackageName, "state", stateKey],
    () => getStateValue(webPackageName, stateKey),
    {
      initialData: value,
    }
  );

  return data;
}
