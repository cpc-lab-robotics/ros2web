import axios from "../axios-config";

export type Page = {
  style: string;
  widgets?: any[];
  bind: Record<string, any>;
  layout?: Record<string, any>;
};

export type Plugin = {
  id: string;
  type: string;
  name: string;
  package_name: string;
  class_name: string;
  module_name: string;
  location: string;
  config: Record<string, any>;
  disable: boolean;
  summary: string;
  version: string;
};


export const getPlugins = async () => {
  const url = `/ros2web/plugins`;
  const response = await axios.get<Array<string>>(url);
  return response.data;
};

export const getPlugin = async (pluginId: string) => {
  const url = `/ros2web/plugin/${pluginId}`;
  const response = await axios.get<Plugin>(url);
  return response.data;
};

export const enablePlugin = async (pluginId: string) => {
  const url = `/ros2web/plugin/enable?id=${pluginId}`;
  const response = await axios.get<Plugin>(url);
  return response.data;
};

export const disablePlugin = async (pluginId: string) => {
  const url = `/ros2web/plugin/disable?id=${pluginId}`;
  const response = await axios.get<Plugin>(url);
  return response.data;
};

export const getPage = async (webPackageName: string, path: string) => {
  const url = `/ros2web/page/${webPackageName}/${path}`;
  const response = await axios.get<Page>(url);
  return response.data;
};

export const getState = async (webPackageName: string) => {
  const response = await axios.get<Record<string, any>>(
    `/ros2web/state/${webPackageName}`
  );
  return response.data;
};

export const getStateValue = async (webPackageName: string, key: string) => {
  const response = await axios.get<any>(`/ros2web/state/${webPackageName}/${key}`);
  return response.data;
};

