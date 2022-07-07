import axios from "axios";

import { getOrLoadRemote } from "@/utils/module-federation";

const loadingPlugin = {};

export const loadWidget = (pluginName: string, widgetName: string) => {
  return async () => {

    
    const url = `/ros2web/widget/${pluginName}/remoteEntry.js`;

    console.log(url)
    
    const remoteName = `ros2web_${pluginName}`;
    try {
      await axios.get<any>(url, {
        headers: {
          'Cache-Control': 'no-cache',
          'Pragma': 'no-cache',
          'Expires': '0',
        },
      });
    } catch (error) {
      return import("@/containers/page/blank");
    }

    if (remoteName in loadingPlugin) {
      let counter = 0;
      while (loadingPlugin[remoteName] && counter < 3) {
        await new Promise((resolve) => setTimeout(resolve, 200));
        counter++;
      }
    } else {
      loadingPlugin[remoteName] = true;
      await getOrLoadRemote(remoteName, "default", url);
      loadingPlugin[remoteName] = false;
    }

    const container = window[remoteName];
    const factory = await container.get(widgetName);
    const Module = factory();
    return Module;
  };
};
