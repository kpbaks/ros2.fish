#!/usr/bin/env python3

import argparse
import os
import sys
import xml.etree.ElementTree as ET
import json
import re
from typing import List, Optional
from dataclasses import dataclass

SCRIPT_NAME: str = os.path.basename(__file__).replace(".py", "")

DEPENDENCY_TAGS: tuple = (
    'buildtool_depend',
    'build_depend',
    'build_export_depend',
    'exec_depend',
    'test_depend',
    'test_export_depend',
    'doc_depend',
    'export_depend',
    'depends',
)

@dataclass
class Author:
    name: str
    email: str
    
@dataclass
class Maintainer:
    name: str
    email: str
    
@dataclass
class Url:
    type: str
    value: str
    
@dataclass
class License:
    type: str
    value: str

@dataclass
class Ros2PackageSpec:
    name: str
    version: str
    description: str 
    maintainer: list[Maintainer]
    author: list[Author]
    license: License
    buildtool_depend: list[str]
    build_depend: list[str]
    build_export_depend: list[str]
    exec_depend: list[str]
    test_depend: list[str]
    test_export_depend: list[str]
    doc_depend: list[str]
    export_depend: list[str]
    depends: list[str]


def gen_args_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog=SCRIPT_NAME, description="")
    parser.add_argument('--version', action='version', version='%(prog)s 0.1.0')
    parser.add_argument('package', nargs='?', help='package.xml file to parse')
    parser.add_argument('-o', '--output', choices=['json'], help='output format')
    parser.add_argument('-d, --dependencies', choices=DEPENDENCY_TAGS, nargs="*", help='dependencies to include in output')

    
    
    return parser

def main(argc: int, argv: list[str]) -> int:
    parser = gen_args_parser()
    args = parser.parse_args(argv[1:])
    

   # path/to/packagepath/to/package.xml.xmlCheck if stdin is a pipe
    if sys.stdin.isatty():
        # Read package.xml file from disk
        if args.package is None:
            parser.print_help()
            return 1

        with open(args.package) as f:
            tree = ET.parse(f)
    else:
        # Read package.xml file from stdin
        tree = ET.parse(sys.stdin)

    root = tree.getroot()

    # Find the package name
    package_name = root.find('name').text

    # Find the package version
    package_version = root.find('version').text

    description = root.find('description').text

        
    def find_dependencies(root, tag: str) -> list[str]:
        return [dep.text for dep in root.findall(tag)]


    # Print the package information
    print("name:", package_name)
    print("version:", package_version)
    print("description:", description)
    
    urls = root.findall('url')
    for url in urls:
        print("url:", url.text)

    maintainers = root.findall('maintainer')
    for maintainer in maintainers:
        print("maintainer:", maintainer.text)

    authors = root.findall('author')
    for author in authors:
        print("author:", author.text)

    licenses = root.findall('license')
    for license in licenses:
        print("license:", license.text)


    
    dependencies = {tag: find_dependencies(root, tag) for tag in DEPENDENCY_TAGS}
    for tag, deps in dependencies.items():
        print(f"{tag}: {deps}")
    
    return 0

if __name__ == "__main__":
    sys.exit(main(len(sys.argv), sys.argv))
