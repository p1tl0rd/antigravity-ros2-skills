#!/usr/bin/env python3
import sys
import asyncio
import json
from typing import List, Dict, Any

try:
    import rclpy
    from rclpy.node import Node
except ImportError:
    sys.stderr.write("Error: rclpy not found. Source your ROS 2 environment first.\n")
    sys.exit(1)

try:
    from mcp.server.fastmcp import FastMCP
except ImportError:
    # Fallback or error if mcp is not installed
    # We will just print a helpful message to stderr and start a dummy server or exit
    sys.stderr.write("Error: 'mcp' python package not found. Please install it with: pip install mcp\n")
    sys.exit(1)

# Initialize FastMCP Server
mcp_server = FastMCP("ros2_context_provider")

def get_ros_graph_info() -> Dict[str, Any]:
    """Helper to query ROS 2 graph using a temporary node."""
    rclpy.init(args=None)
    node = rclpy.create_node('_agent_context_scanner')
    
    try:
        node_names = node.get_node_names_and_namespaces()
        topic_names_and_types = node.get_topic_names_and_types()
        service_names_and_types = node.get_service_names_and_types()
        
        nodes_info = []
        for name, namespace in node_names:
            full_name = f"{namespace}/{name}".replace("//", "/")
            nodes_info.append({"name": name, "namespace": namespace, "full_name": full_name})

        topics_info = []
        for name, types in topic_names_and_types:
            topics_info.append({"name": name, "types": types})

        services_info = []
        for name, types in service_names_and_types:
            services_info.append({"name": name, "types": types})

        return {
            "nodes": nodes_info,
            "topics": topics_info,
            "services": services_info
        }
    finally:
        node.destroy_node()
        rclpy.shutdown()

@mcp_server.resource("ros2://graph")
def get_graph() -> str:
    """Returns the current ROS 2 graph (nodes, topics, services) as JSON."""
    try:
        data = get_ros_graph_info()
        return json.dumps(data, indent=2)
    except Exception as e:
        return json.dumps({"error": str(e)})

@mcp_server.tool()
def list_nodes() -> str:
    """Lists all active ROS 2 nodes."""
    graph = get_ros_graph_info()
    return json.dumps(graph["nodes"], indent=2)

@mcp_server.tool()
def list_topics() -> str:
    """Lists all active ROS 2 topics."""
    graph = get_ros_graph_info()
    return json.dumps(graph["topics"], indent=2)

if __name__ == "__main__":
    mcp_server.run()
