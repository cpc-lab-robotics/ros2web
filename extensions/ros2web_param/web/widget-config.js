const path = require("path");

const widget = {
  name: "param",
  port: 3001,
  proxy: {
    "/api": "http://localhost:8080",
  },
  path: path.join(__dirname, "..", "ros2web_param", "data", "widgets"),
  publicPath: "auto",
  exposes: {
    "Config": "./src/components/ParamConfig",
  }
};

module.exports = widget;