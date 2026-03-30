from __future__ import annotations

from rclpy.node import Node
from rclpy.parameter import Parameter


def declare_required_parameters(node: Node, specs: list[tuple[str, Parameter.Type]]) -> None:
    for name, parameter_type in specs:
        node.declare_parameter(name, parameter_type)


def require_parameter(node: Node, name: str):
    value = node.get_parameter(name).value
    if value is None:
        raise RuntimeError(f"Missing required parameter '{name}' for node '{node.get_name()}'.")
    return value


def require_bool(node: Node, name: str) -> bool:
    return bool(require_parameter(node, name))


def require_float(node: Node, name: str) -> float:
    return float(require_parameter(node, name))


def require_int(node: Node, name: str) -> int:
    return int(require_parameter(node, name))


def require_str(node: Node, name: str) -> str:
    return str(require_parameter(node, name))
