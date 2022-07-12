from types import ModuleType

import importlib

def reload_module(module, package_name, reloaded):
    """Recursively reload modules."""
    importlib.reload(module)
    reloaded[module.__name__] = module
    for attribute_name in dir(module):
        attribute = getattr(module, attribute_name)
        if type(attribute) is ModuleType:
            if attribute.__name__.startswith(package_name) and attribute.__name__ not in reloaded:
                reload_module(attribute, package_name, reloaded)
                