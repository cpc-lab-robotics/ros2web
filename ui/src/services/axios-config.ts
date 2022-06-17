import axios from "axios";

const baseURL = "/api/ros2web";
export const instance = axios.create({
  baseURL,
});

export default instance;
