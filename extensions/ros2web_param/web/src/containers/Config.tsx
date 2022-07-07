import React, { useEffect, useState } from "react";
import axios from "axios";
import { useMutation, useQueries, useQueryClient } from "react-query";

import { Param } from "@/models/param";
import ParamConfig from "@/components/ParamConfig";

const getParam = async (nodeName: string, paramName: string) => {
  const url = `/api/ros2web/param/get`;
  const response = await axios.post<Param>(url, {
    node_name: nodeName,
    param_name: paramName,
  });
  return response.data;
};

const setParam = async (param: Param) => {
  const url = `/api/ros2web/param/set`;
  const response = await axios.post<Param>(url, param);
  return response.data;
};

function useParams(
  nodeName: string,
  paramNames: Array<string>
): Array<Param> {
  const results = useQueries(
    paramNames.map((paramName: string) => ({
      queryKey: ["ros2web_param", nodeName, paramName],
      queryFn: () => getParam(nodeName, paramName),
      staleTime: 0,
      cacheTime: 5 * 60 * 1000,
    }))
  );

  const params: Array<Param> = [];
  results.forEach((result, index) => {
    const { data } = result;
    if (data !== undefined) {
      params.push(data);
    }
  });
  
  return params;
}

export type Props = {
  node_name: string;
  param_names: string[];
};

export default function ConfigWidget(props: Props) {
  const { node_name, param_names } = props;

  const queryClient = useQueryClient();

  const params = useParams(node_name, param_names);

  const mutation = useMutation((param: Param) => setParam(param), {
    onSuccess: (param) => {
      queryClient.setQueryData<Param>(
        ["ros2web_param", param.nodeName, param.name],
        param
      );
    },
  });

  const changeHandler = (param: Param) => {
    mutation.mutate(param);
  };

  return <ParamConfig params={params} on_change={changeHandler} />;
}
