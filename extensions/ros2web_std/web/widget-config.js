const path = require("path");

const widget = {
  name: "std",
  port: 3001,
  path: path.join(__dirname, "..", "ros2web_std", "data", "widgets"),
  publicPath: "auto",
  exposes: {
    "List": "./src/components/List",
    "Button": "./src/components/Button",
  }
};

module.exports = widget;