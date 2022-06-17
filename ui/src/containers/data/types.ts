export type IndexData = {
  id: string;
  primary: string;
  secondary?: string;
};

export type ListData = {
  items: Array<ListItemData>;
};

export type ListItemData = {
  header: string;
  rows: Array<TableRowData>;
};

export type TableData = {
  headers: Array<string>;
  rows: Array<TableRowData>;
};

export type TableRowData = {
  columns: Array<TableColumnData>;
};

export type TableColumnData = {
  span: number;
  type: string;
  value: string | number;
  table?: TableData;
};

export type GraphData = {
  nodes: Array<GraphNode>;
};

export type GraphNode = {
  id: string;
  style: string;
  name: string;
  group?: string;
  subGroup?: string;
  inbounds?: GraphLink[];
  outbounds?: GraphLink[];
};

export type GraphLink = {
  name: string;
  type: string;
};
