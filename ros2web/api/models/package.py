from typing import Any, List, Dict, Optional

from dataclasses import dataclass, field


# https://www.ros.org/reps/rep-0149.html

@dataclass
class Dependency:
    name: str
    version_lt: Optional[str] = field(default=None)
    version_lte: Optional[str] = field(default=None)
    version_eq: Optional[str] = field(default=None)
    version_gte: Optional[str] = field(default=None)
    version_gt: Optional[str] = field(default=None)
    condition: Optional[str] = field(default=None)


@dataclass
class Export:
    tagname: str
    attributes: Optional[Dict[str, str]] = field(default=None)
    content: Optional[str] = field(default=None)


@dataclass
class Person:
    name: str
    email: Optional[str] = field(default=None)


@dataclass
class Url:
    url: str
    type: Optional[str] = field(default=None)


@dataclass
class PackageManifest:
    filename: str
    package_format: int

    name: str
    version: str  # MAJOR.MINOR.PATCH
    description: str
    maintainers: List[Person] = field(default_factory=list)
    licenses: list = field(default_factory=list)

    urls: Optional[List[Url]] = field(default=None)
    authors: Optional[List[Person]] = field(default=None)
    
    build_depends: Optional[List[Dependency]] = field(default=None)
    build_export_depends: Optional[List[Dependency]] = field(default=None)
    buildtool_depends: Optional[List[Dependency]] = field(default=None)
    buildtool_export_depends: Optional[List[Dependency]] = field(default=None)
    exec_depends: Optional[List[Dependency]] = field(default=None)
    depends: Optional[List[Dependency]] = field(default=None)
    doc_depends: Optional[List[Dependency]] = field(default=None)
    test_depends: Optional[List[Dependency]] = field(default=None)
    conflicts: Optional[List[Dependency]] = field(default=None)
    replaces: Optional[List[Dependency]] = field(default=None)
    group_depends: Optional[List[Dependency]] = field(default=None)
    member_of_groups: Optional[List[Dependency]] = field(default=None)
    
    exports: Optional[List[Export]] = field(default=None)

@dataclass
class Package:
    name: str
    executables: Optional[List[str]] = field(default=None)
    manifest: Optional[List[PackageManifest]] = field(default=None)