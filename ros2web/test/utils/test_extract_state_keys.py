import pytest
import os.path

from ros2web.utilities import open_yaml_file, extract_state_keys


def test_extract_state_key():
    config = open_yaml_file('ros2web', 'example.yml')
    page = config.get('page')
    widgets = page.get('widgets')
    
    for widget in widgets:
        name = widget.get('name')
        
        props = widget.get('props', {})
        state_keys = set()
        extract_state_keys(props, state_keys)
        print(name, state_keys)
            
    
    
