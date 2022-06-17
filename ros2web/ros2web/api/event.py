from typing import Any
from typing import Optional, NamedTuple

class WidgetEvent(NamedTuple):
    widget_id: str
    type: str
    value: Optional[Any]