// https://tjkendev.github.io/procon-library/python/geometry/graham_scan.html

function cross3(a: number[], b: number[], c: number[]) {
  return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
}

export function grahamScan(points: number[][]): number[][] {
  let ps = points.sort((a, b) => a[0] - b[0]);
  // let ps = points.sort((a: Point, b: Point) => a.x - b.x);
  const qs: number[][] = [];
  const N = ps.length;
  ps.forEach((p) => {
    while (qs.length > 1 && cross3(qs[qs.length - 1], qs[qs.length - 2], p) > 0)
      qs.pop();
    qs.push(p);
  });
  const t = qs.length;
  for (let i = N - 2; i > -1; i--) {
    let p = ps[i];
    while (qs.length > t && cross3(qs[qs.length - 1], qs[qs.length - 2], p) > 0)
      qs.pop();
    qs.push(p);
  }

  qs.pop();
  return qs;
}
