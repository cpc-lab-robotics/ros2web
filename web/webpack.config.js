const HtmlWebpackPlugin = require("html-webpack-plugin");
const ReactRefreshWebpackPlugin = require("@pmmmwh/react-refresh-webpack-plugin");
const ReactRefreshTypeScript = require("react-refresh-typescript");
const { ModuleFederationPlugin } = require("webpack").container;
const TsconfigPathsPlugin = require("tsconfig-paths-webpack-plugin");
const path = require("path");
const deps = require("./package.json").dependencies;

const isDevelopment = process.env.NODE_ENV !== "production";

module.exports = {
  mode: isDevelopment ? "development" : "production",
  entry: "./src/index",
  devServer: {
    hot: true,
    historyApiFallback: true,
    port: 3000,
    proxy: {
      "/subscription": {
        target: "ws://localhost:8080",
        ws: true,
      },
      "/api": "http://localhost:8080",
      "/ros2web/widget": "http://localhost:8080",
    },
  },
  optimization: {
    runtimeChunk: isDevelopment
      ? {
          name: "runtime",
        }
      : false,
  },
  output: {
    path: path.join(__dirname, "..", "ros2web", "ros2web", "data", "public"),
    publicPath: isDevelopment ? "/" : "/ros2web/",
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
        test: /\.module.scss$/,
        use: [
          "style-loader",
          {
            loader: "css-loader",
            options: {
              sourceMap: isDevelopment,
              modules: true,
              importLoaders: 2,
            },
          },
          {
            loader: "sass-loader",
            options: {
              sourceMap: isDevelopment,
            },
          },
        ],
      },
      {
        test: /\.scss$/,
        use: ["style-loader", "css-loader", "sass-loader"],
      },

      {
        test: /\.css$/,
        use: ["style-loader", "css-loader"],
      },
    ],
  },
  resolve: {
    extensions: [".ts", ".tsx", ".js", ".json"],
    plugins: [new TsconfigPathsPlugin()],
  },
  plugins: [
    new ModuleFederationPlugin({
      name: "ros2web",
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
      template: "./src/index.html",
    }),
    isDevelopment && new ReactRefreshWebpackPlugin(),
  ].filter(Boolean),
};
