const HtmlWebpackPlugin = require("html-webpack-plugin");
const ReactRefreshWebpackPlugin = require("@pmmmwh/react-refresh-webpack-plugin");
const ReactRefreshTypeScript = require("react-refresh-typescript");
const { ModuleFederationPlugin } = require("webpack").container;
const TsconfigPathsPlugin = require("tsconfig-paths-webpack-plugin");
const path = require("path");
const deps = require("./package.json").dependencies;
const widget = require("./widget-config");

const isDevelopment = process.env.NODE_ENV !== "production";

module.exports = {
  mode: isDevelopment ? "development" : "production",
  entry: "./src/index",
  devServer: {
    hot: true,
    historyApiFallback: true,
    port: widget.port,
    proxy: widget.proxy,
  },
  optimization: {
    runtimeChunk: isDevelopment
      ? {
          name: "runtime",
        }
      : false,
  },
  output: {
    path: widget.path,
    publicPath: widget.publicPath,
  },
  ...(isDevelopment ? { devtool: "source-map" } : {}),
  module: {
    rules: [
      {
        test: /\.(ts|tsx)$/,
        use: [
          {
            loader: "babel-loader",
            options: {
              plugins: [
                isDevelopment && require.resolve("react-refresh/babel"),
              ].filter(Boolean),
              presets: ["@babel/preset-env", "@babel/react"],
            },
          },
          {
            loader: "ts-loader",
            options: {
              getCustomTransformers: () => ({
                before: [isDevelopment && ReactRefreshTypeScript()].filter(
                  Boolean
                ),
              }),
              transpileOnly: isDevelopment,
              configFile: path.resolve(__dirname, "tsconfig.json"),
            },
          },
        ],
      },
      {
        test: /\.scss$/,
        use: ["style-loader", "css-loader", "sass-loader"],
      },
    ],
  },
  resolve: {
    extensions: [".ts", ".tsx", ".js", ".json"],
    plugins: [new TsconfigPathsPlugin()],
  },
  plugins: [
    new ModuleFederationPlugin({
      name: `ros2web_${widget.name}`,
      filename: "remoteEntry.js",
      exposes: widget.exposes,
      shared: {
        react: {
          requiredVersion: deps.react,
          import: "react", // the "react" package will be used a provided and fallback module
          shareKey: "react", // under this name the shared module will be placed in the share scope
          shareScope: "default", // share scope with this name will be used
          singleton: true, // only a single version of the shared module is allowed
        },
        "react-dom": {
          requiredVersion: deps["react-dom"],
          singleton: true, // only a single version of the shared module is allowed
        },
        "@mui/material": {
          requiredVersion: deps["@mui/material"],
          singleton: true,
        },
        "@mui/icons-material": {
          requiredVersion: deps["@mui/icons-material"],
          singleton: true,
        },
        "@emotion/react": {
          requiredVersion: deps["@emotion/react"],
          singleton: true,
        },
        "@emotion/styled": {
          requiredVersion: deps["@emotion/styled"],
          singleton: true,
        },
      },
    }),
    new HtmlWebpackPlugin({
      template: "./index.html",
    }),
    isDevelopment && new ReactRefreshWebpackPlugin(),
  ].filter(Boolean),
};
