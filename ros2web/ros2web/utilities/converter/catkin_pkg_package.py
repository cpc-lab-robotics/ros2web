from typing import Dict, Any

from catkin_pkg.package import Url, License, Dependency, Export, Package, Person
from catkin_pkg.group_dependency import GroupDependency
from catkin_pkg.group_membership import GroupMembership

def _object_to_dict(obj):
    obj_dict = {}
    for attr_name in dir(obj):
        if attr_name.startswith("_"):
            continue
        attr = getattr(obj, attr_name)

        if callable(attr):
            continue
        obj_dict[attr_name] = _to_dict(attr)
    return obj_dict

def _to_dict(obj):
    if isinstance(obj, Package):
        return _object_to_dict(obj)
    elif isinstance(obj, Dependency):
        return _object_to_dict(obj)
    elif isinstance(obj, GroupDependency):
       return _object_to_dict(obj)
    elif isinstance(obj, GroupMembership):
       return _object_to_dict(obj)
    elif isinstance(obj, Export):
        return _object_to_dict(obj)
    elif isinstance(obj, Person):
        return _object_to_dict(obj)
    elif isinstance(obj, Url):
        return _object_to_dict(obj)
    elif isinstance(obj, License):
        return str(obj)
    elif isinstance(obj, list):
        return [_to_dict(a) for a in obj]
    elif isinstance(obj, set):
        return {_to_dict(a) for a in obj}
    elif isinstance(obj, dict):
        return {k:_to_dict(v) for k, v in obj.items()}
    else:
        return obj

def convert_catkin_pkg_to_dict(package: Package)->Dict[str, Any]:
    package_dict = _to_dict(package)

    depend = {}
    set_dict = {}
    tags =  ['build_depends', 'build_export_depends', 'exec_depends']

    for tag in tags:
        depend[tag] = [{'name': d["name"], 'd': d} for d in package_dict[tag]]
        set_dict[tag] = set([d["name"] for d in package_dict[tag]])

    same_name = set_dict['build_depends'].intersection(
        set_dict['build_export_depends'], set_dict['exec_depends'])

    for tag in tags:
        package_dict[tag] = [d['d']
                        for d in depend[tag] if d['name'] not in same_name]

    package_dict["depends"] = [d['d']
                         for d in depend['build_depends'] if d['name'] in same_name]

    # del package_dict["filename"]
    del package_dict["version_compatibility"]
    
    return package_dict
