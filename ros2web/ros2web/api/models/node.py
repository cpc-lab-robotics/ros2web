from dataclasses import dataclass, field
from typing import Optional
from typing import List, Any

from .topic import Topic
from .service import Service
from .action import Action
from .data import Data

@dataclass
class NodeInterface:
    node_name: str
    subscribers: Optional[List[Topic]] = field(default=None)
    publishers: Optional[List[Topic]] = field(default=None)
    service_servers: Optional[List[Service]] = field(default=None)
    service_clients: Optional[List[Service]] = field(default=None)
    action_servers: Optional[List[Action]] = field(default=None)
    action_clients: Optional[List[Action]] = field(default=None)
    
@dataclass
class Node(Data):
    name: str
    namespace: str
    full_name: str
    interface: Optional[NodeInterface] = field(default=None)
    