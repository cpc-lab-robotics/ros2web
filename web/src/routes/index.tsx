import React from "react";

import { BrowserRouter, Routes, Route, Navigate } from "react-router-dom";
import WebPackage from "@/containers/page";
import Settings from "@/containers/settings";


const AppRoutes = () => {
  return (
    <BrowserRouter>
      <Routes>
        <Route path="/" element={<Navigate to={"/ros2web"} />} />
        <Route path="/ros2web" element={<Settings />} />
        <Route path="/ros2web/:webPackageName/*" element={<WebPackage />} />
      </Routes>
    </BrowserRouter>
  );
};

export default AppRoutes;
