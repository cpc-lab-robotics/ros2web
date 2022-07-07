const path = require("path");

const widget = {
  name: "@plugin_name",
  port: 3001,
  proxy: {
    "/api": "http://localhost:8080",
  },
  path: path.join(__dirname, "..", "@package_name", "data", "widgets"),
  publicPath: "auto",
  exposes: {
    "Widget": "./src/Widget",
  }
};

module.exports = widget;