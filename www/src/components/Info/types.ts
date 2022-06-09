
export type InfoData = {
  title: string;
  items: Array<KeyValueWithHeader | KeyValue>;
};

export type KeyValueWithHeader = {
  header: string;
  keyValues: KeyValue [];
};

export type KeyValue = {
  key: string;
  value: any; 
};
