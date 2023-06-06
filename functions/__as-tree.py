#!/usr/bin/env python3

import argparse
import functools
import itertools
import os
import sys
from dataclasses import dataclass
from typing import Callable, List, Optional

SCRIPT_NAME: str = os.path.basename(__file__).replace(".py", "")

RED = "\033[31m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
BLUE = "\033[34m"
MAGENTA = "\033[35m"
CYAN = "\033[36m"
NC = "\033[0m"

cmap = {
    0: RED,
    1: GREEN,
    2: YELLOW,
    3: BLUE,
    4: MAGENTA,
    5: CYAN,
}

# Taken from: https://en.wikipedia.org/wiki/Box-drawing_character
VERTICAL_BAR = "│"
TEE = "├"
END = "└"
START = "─"


@dataclass
class Node:
    name: str
    children: List["Node"]
    level: int
    metadata: Optional[str] = None

    @property
    def is_leaf(self) -> bool:
        return len(self.children) == 0

    @property
    def leaves(self) -> List["Node"]:
        if self.is_leaf:
            return [self]
        else:
            return list(
                itertools.chain.from_iterable(child.leaves for child in self.children)
            )


def term_tree_view(root: Node, indent: int, formatter: Callable[[Node], str]) -> None:
    def gen_indentation(number_of_remaining_children_for_each_parent: List[int]) -> str:
        def gen_indentation_inner(
            number_of_remaining_children_for_each_parent: List[int], acc: str
        ) -> str:
            l = len(number_of_remaining_children_for_each_parent)
            if l == 0:
                return acc + ""
            elif l == 1:
                head = number_of_remaining_children_for_each_parent[0]
                if head == 0:
                    return acc + END + (START * (indent - 2)) + " "
                elif head > 0:
                    return acc + TEE + (START * (indent - 2)) + " "
                else:
                    raise Exception("unreachable")
            else:
                head = number_of_remaining_children_for_each_parent[0]
                tail = number_of_remaining_children_for_each_parent[1:]
                if head == 0:
                    return acc + (" " * indent) + gen_indentation_inner(tail, acc)
                elif head > 0:
                    return (
                        acc
                        + VERTICAL_BAR
                        + (" " * (indent - 1))
                        + gen_indentation_inner(tail, acc)
                    )
                else:
                    raise Exception("unreachable")


            # match number_of_remaining_children_for_each_parent:
            #     case []:
            #         return acc + ""
            #     case [head] if head == 0:
            #         return acc + END + (START * (indent - 2)) + " "
            #     case [head] if head > 0:
            #         return acc + TEE + (START * (indent - 2)) + " "
            #     case [head, *tail] if head == 0:
            #         return acc + (" " * indent) + gen_indentation_inner(tail, acc)
            #     case [head, *tail] if head > 0:
            #         return (
            #             acc
            #             + VERTICAL_BAR
            #             + (" " * (indent - 1))
            #             + gen_indentation_inner(tail, acc)
            #         )
            #     case _:
            #         raise Exception("unreachable")

        return gen_indentation_inner(number_of_remaining_children_for_each_parent, "")

    def print_node(
        node: Node, number_of_remaining_children_for_each_parent: Optional[List[int]]
    ) -> None:
        if number_of_remaining_children_for_each_parent is None:
            number_of_remaining_children_for_each_parent = []

        indentation: str = gen_indentation(number_of_remaining_children_for_each_parent)
        node_formatted: str = formatter(node)
        print(f"{indentation}{node_formatted}")

        number_of_children = len(node.children)
        for i, child in enumerate(node.children):
            print_node(
                child,
                number_of_remaining_children_for_each_parent
                + [number_of_children - i - 1],
            )

    print_node(root, None)


def compose(*functions):
    return functools.reduce(lambda f, g: lambda x: f(g(x)), functions)


def gen_args_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog=SCRIPT_NAME, description="")
    parser.add_argument("-d", "--delimiter", type=str, default="/", help="delimiter")
    parser.add_argument("--wrap-metadata-in-parens", action="store_true")
    parser.add_argument(
        "--metadata-delimiter", type=str, default=" ", help="metadata delimiter"
    )
    parser.add_argument(
        "-r", "--right-align-metadata", action="store_true", help="right align metadata"
    )
    parser.add_argument("-i", "--indent", type=int, default=4, help="indent in spaces")
    parser.add_argument("-s", "--sort", action="store_true", help="sort")
    parser.add_argument("-c", "--count", action="store_true", help="count")
    parser.add_argument(
        "--color", action="store_true", help="apply a different color to each level"
    )
    # parser.add_argument(
    #     "--fs",
    #     action="store_true",
    #     help="interpret input as filesystem paths, and highlight the last path component",
    # )
    return parser


def main(argc: int, argv: List[str]) -> int:
    parser = gen_args_parser()
    args = parser.parse_args(argv[1:])

    assert args.indent >= 0, f"Indent must be >= 0, got {args.indent}"
    assert args.delimiter != "", "--delimiter must not be empty"
    assert args.metadata_delimiter != "", "--metadata-delimiter must not be empty"
    assert (
        args.delimiter != args.metadata_delimiter
    ), "--delimiter and --metadata-delimiter must be different"

    mutations = compose(
        lambda line: line.strip(),
        lambda line: line[2:]
        if line[:2] == "./"
        else line,  # Remove leading `./` if present
    )
    lines: List[str] = [mutations(line) for line in sys.stdin.readlines()]

    filters = [lambda line: line not in ["", ".", ".."]]

    filtered_lines: List[str] = [
        line for line in lines if all(filter(line) for filter in filters)
    ]

    list_of_metadata = []
    for split in [line.split(args.metadata_delimiter) for line in filtered_lines]:
        if len(split) > 1:
            list_of_metadata.append(args.metadata_delimiter.join(split[1:]))
        else:
            list_of_metadata.append(None)

    # Split lines by delimiter and remove empty strings
    filtered_lines_without_metadata = [
        line.split(args.metadata_delimiter)[0] for line in filtered_lines
    ]
    lines_split_by_delimiter = [
        line.split(args.delimiter) for line in filtered_lines_without_metadata
    ]
    list_of_segments = [
        [segment for segment in line if segment != ""]
        for line in lines_split_by_delimiter
    ]

    assert len(list_of_segments) == len(
        list_of_metadata
    ), f"Number of lines and number of metadata must be the same, but got {len(list_of_segments)} and {len(list_of_metadata)} respectively"
    # Create a tree
    root = Node(name="root", children=[], level=0)
    for line, metadata in zip(list_of_segments, list_of_metadata):
        node = root
        for segment in line:
            # Check if the segment already exists
            for child in node.children:
                if child.name == segment:
                    node = child
                    break
            else:
                # If not, create a new node
                new_node = Node(
                    name=segment,
                    children=[],
                    level=node.level + 1,
                    metadata=metadata
                )
                node.children.append(new_node)
                node = new_node

    def formatter(node: Node) -> str:
        metadata: str = ""
        if node.is_leaf and node.metadata is not None:
            if args.wrap_metadata_in_parens:
                metadata = f" ({node.metadata})"
            else:
                metadata = f" {node.metadata}"
        if args.color:
            return cmap[node.level % len(cmap)] + node.name + NC + metadata
        else:
            return node.name + metadata

    # Print the trees
    for node in root.children:
        term_tree_view(node, args.indent, formatter)

    if args.count:
        print("")
        print(f"count: {len(root.leaves) + 1}")

    return 0


if __name__ == "__main__":
    sys.exit(main(len(sys.argv), sys.argv))
