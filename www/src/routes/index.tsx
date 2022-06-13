import React, { Suspense } from "react";

import {
  BrowserRouter,
  Routes,
  Route,
  Navigate,
  useParams,
} from "react-router-dom";
import WebPackage from "@/containers/web-package";
import Settings from "@/containers/settings";

const AppRoutes = () => {
  return (
    <BrowserRouter>
      <Routes>
        <Route path="/" element={<Navigate to="/ros2web" replace />} />
        <Route path="/ros2web" element={<Settings />} />
        <Route path="/ros2web/:webPackageName/*" element={<WebPackage />} />
      </Routes>
    </BrowserRouter>
  );
};

export default AppRoutes;
