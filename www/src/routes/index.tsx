import React, { Suspense } from "react";

import {
  BrowserRouter,
  Routes,
  Route,
  Navigate,
  useParams,
} from "react-router-dom";
import WebPackage from "@/containers/web-package";


const AppRoutes = () => {
  return (
    <BrowserRouter>
      <Routes>
        <Route path="/" element={<Navigate to="/ui/" replace />} />
        <Route
          path="/ui"
          element={<main style={{ padding: "1rem" }}>ui</main>}
        />
        <Route path="/ui/:webPackageName" element={<WebPackage />} />
        <Route
          path="*"
          element={
            <main style={{ padding: "1rem" }}>
              <p>There's nothing here!</p>
            </main>
          }
        />
      </Routes>
    </BrowserRouter>
  );
};

export default AppRoutes;
