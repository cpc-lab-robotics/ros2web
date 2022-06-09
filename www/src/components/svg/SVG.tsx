import React, { useEffect, useRef, useState } from "react";

const Context = React.createContext(null);

type SVGProps = {
  width: number;
  height: number;
  children?: any;
}

export default function SVG({ width, height, children }: SVGProps) {
  const svgRef = useRef(null);
  const [svg, setSvg] = useState(null);

  useEffect(() => setSvg(svgRef.current), []);
    
  const boxSize = [-width / 2, -height / 2, width, height]
  
  return (
    <svg
      ref={svgRef}
      width={width}
      height={height}
      viewBox={boxSize.toString()}
    >
      
      <Context.Provider value={svg}>{children}</Context.Provider>
    </svg>
  );
}

export function useSvg() {
  return React.useContext(Context);
}
