import axios from "../axios-config";

export const getStateValue = async (webPackageName: string, key: string) => {
  const response = await axios.get<any>(`/${webPackageName}/state/${key}`);
  return response.data;
};

export const getState = async (webPackageName: string) => {
  const response = await axios.get<any>(`/${webPackageName}/state`);
  return response.data;
};

export const getInfo = async (webPackageName: string) => {
  const response = await axios.get<any>(`/${webPackageName}/info`);
  return response.data;
};