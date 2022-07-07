from typing import Dict, Set, Optional

import re

def extract_state_key(value:str) -> Optional[str]:
    m = re.match(r'^\$\{(.+)\}$', value)
    return m.group(1) if m is not None else None        
        
def extract_state_keys(props, state_keys: Set):
    if isinstance(props, str):
        state_key = extract_state_key(props)
        if state_key:
            state_keys.add(state_key)
    elif isinstance(props, dict):
        [extract_state_keys(v, state_keys) for v in props.values()]
    elif isinstance(props, list):
        [extract_state_keys(v, state_keys) for v in props]