{
  "name": "@package_name",
  "version": "0.0.1",
  "private": true,
  "devDependencies": {
    "@@babel/core": "^7.18.5",
    "@@babel/preset-env": "^7.18.2",
    "@@babel/preset-react": "^7.17.12",
    "@@pmmmwh/react-refresh-webpack-plugin": "^0.5.7",
    "@@types/react": "^18.0.9",
    "@@types/react-dom": "^18.0.4",
    "babel-loader": "^8.2.5",
    "css-loader": "^6.7.1",
    "html-webpack-plugin": "^5.5.0",
    "react-refresh": "^0.14.0",
    "react-refresh-typescript": "^2.0.5",
    "sass": "^1.52.3",
    "sass-loader": "^13.0.0",
    "serve": "^13.0.2",
    "style-loader": "^3.3.1",
    "ts-loader": "^9.3.0",
    "tsconfig-paths-webpack-plugin": "^3.5.2",
    "typescript": "^4.7.3",
    "webpack": "^5.72.1",
    "webpack-cli": "^4.10.0",
    "webpack-dev-server": "^4.8.1"
  },
  "scripts": {
    "start": "webpack-cli serve",
    "build": "NODE_ENV=production webpack",
    "clean": "rm -rf ../@package_name/data/widgets"
  },
  "dependencies": {
    "@@emotion/react": "^11.9.3",
    "@@emotion/styled": "^11.9.3",
    "@@mui/icons-material": "^5.8.4",
    "@@mui/material": "^5.8.5",
    "react": "^18.0.0",
    "react-dom": "^18.0.0"
  }
}
