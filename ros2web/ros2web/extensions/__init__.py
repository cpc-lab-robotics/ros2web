from typing import NamedTuple
from pathlib import Path

class Extension(NamedTuple):
    name: str
    class_name: str
    module_name: str
    location: str

location = Path(__file__).parent.parent.parent

# extension = Extension(name='example', 
#           class_name='Example', 
#           module_name='ros2web.extensions.example',
#           location=location)
# extensions = [extension]

extensions = []