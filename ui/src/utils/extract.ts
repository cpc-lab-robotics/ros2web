
export function extractStateKey(value: string): string | null {
  if (typeof value === "string" && value.startsWith("$")) {
    const m = value.match(/^\${(.+)}$/);
    const state_key = m && m.length > 1 ? m[1] : null;
    return state_key;
  } else {
    return null;
  }
}
