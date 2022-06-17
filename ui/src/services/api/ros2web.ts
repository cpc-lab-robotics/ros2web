import axios from "../axios-config";

export type Page = {
  ui: {
    layout?: any;
    widgets?: any[];
  },
  bind: Record<string, any>
}

export const getPage = async (webPackageName: string, path: string) => {
  const url = `/page/${webPackageName}/${path}`;
  const response = await axios.get<Page>(url);
  return response.data;
};

export const getState = async (webPackageName: string) => {
  const response = await axios.get<Record<string, any>>(`/state/${webPackageName}`);
  return response.data;
};

export const getStateValue = async (webPackageName: string, key: string) => {
  const response = await axios.get<any>(`/state/${webPackageName}/${key}`);
  return response.data;
};
