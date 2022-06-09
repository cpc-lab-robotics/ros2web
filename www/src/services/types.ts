

export const QUERY_STATUS = {
  LOADING: 'loading',
  SUCCESS: 'success',
  IDLE: 'idle',
  ERROR: "error"
} as const;
export type QueryStatus = typeof QUERY_STATUS[keyof typeof QUERY_STATUS];
