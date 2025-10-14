import json
from server import (
    get_list_topics, get_topic_type, get_topic_message, get_topic_publishers, get_topic_subscribers,get_echo_topic_once)
import matplotlib.patches as mpatches
from config import TOPIC_COLOR, PUBLISHER_COLOR, SUBSCRIBER_COLOR, NODE_SIZE, TOPIC_SIZE, PLOT_WIDTH,PLOT_HEIGHT
import matplotlib.pyplot as plt
import networkx as nx
import tempfile
import json


# ---------- JSON Pretty Formatter ----------
def pretty_format(data) -> str:
    """Format dict/list objects in a readable way"""
    try:
        return json.dumps(data, indent=8, ensure_ascii=False)
    except Exception:
        return str(data)
    
def parse_json(result_str):
    """Convert pretty_format() output back into Python objects if needed"""
    try:
        return json.loads(result_str)
    except Exception:
        return []
    
def extract_nodes(data, key):
    """Extract list of node names from topic dict"""
    if not isinstance(data, dict):
        return []
    return data.get(key, [])  # key = "publishers" or "subscribers"
    
def view_list_topics():
    try:
        result = get_list_topics()
        return result.get("topics", [])
    except Exception as e:
        print(f"Error fetching topics: {e}")
        return []

def get_topic_operations() -> list[str]:
    return ["View Type", "View Message Definition", "View Publishers", "View Subscribers", "Echo"]

def execute_topic_operation(topic: str, operation: str):
    if not topic:
        return "Please select a topic."
    if not operation:
        return "Please select an operation."

    # try:
    if operation == "View Type":
        result = get_topic_type(topic)
    elif operation == "View Message Definition":
        topic_type = get_topic_type(topic).get("type", "")
        if not topic_type:
            return f"Could not resolve message type for {topic}"
        result = get_topic_message(topic_type)
    elif operation == "View Publishers":
        result = get_topic_publishers(topic)
    elif operation == "View Subscribers":
        result = get_topic_subscribers(topic)
    elif operation =="Echo":
        topic_type = get_topic_type(topic).get("type", "")
        if not topic_type:
            return f"Could not resolve message type for {topic}"
        result = get_echo_topic_once(topic,topic_type)
    else:
        return "Unknown operation."

    return pretty_format(result)
    # except Exception as e:
    #     return f"Error executing {operation} on {topic}: {e}"
    

def build_ros_graph_snapshot(selected_topics=None):
    G = nx.DiGraph()
    topics = selected_topics if selected_topics else view_list_topics()
    
    pub_y, topic_y, sub_y = 0, 0, 0
    y_step = 1.5
    pos = {}

    for topic in topics:
        # --- Publishers ---
        pub_data = parse_json(execute_topic_operation(topic, "View Publishers"))
        pubs = extract_nodes(pub_data, "publishers")
        for pub in pubs:
            G.add_node(pub, color=PUBLISHER_COLOR, node_size=NODE_SIZE)
            G.add_edge(pub, topic)
            pos[pub] = (-2, pub_y)
            pub_y += y_step

        # --- Topic Node ---
        G.add_node(topic, color=TOPIC_COLOR, node_size=TOPIC_SIZE)
        pos[topic] = (0, topic_y)
        topic_y += y_step

        # --- Subscribers ---
        sub_data = parse_json(execute_topic_operation(topic, "View Subscribers"))
        subs = extract_nodes(sub_data, "subscribers")
        for sub in subs:
            G.add_node(sub, color=SUBSCRIBER_COLOR, node_size=NODE_SIZE)
            G.add_edge(topic, sub)
            pos[sub] = (2, sub_y)
            sub_y += y_step

    # Draw graph
    plt.figure(figsize=(PLOT_WIDTH, PLOT_HEIGHT))
    node_colors = [G.nodes[n].get("color", "blue") for n in G.nodes()]
    node_sizes = [G.nodes[n].get("node_size", NODE_SIZE) for n in G.nodes()]

    nx.draw(
        G, pos, with_labels=True,
        node_size=node_sizes,
        node_color=node_colors,
        font_size=12,
        font_weight="bold",
        arrows=True,
        arrowsize=5,
        width=1.0,
        connectionstyle='arc3,rad=0.1'
    )

    # Legend

    topic_patch = mpatches.Patch(color=TOPIC_COLOR, label='Topic')
    publisher_patch = mpatches.Patch(color=PUBLISHER_COLOR, label='Publisher Node')
    subscriber_patch = mpatches.Patch(color=SUBSCRIBER_COLOR, label='Subscriber Node')
    plt.legend(handles=[topic_patch, publisher_patch, subscriber_patch], loc='upper right', fontsize=6)

    tmp_png = tempfile.NamedTemporaryFile(delete=False, suffix=".png")
    plt.savefig(tmp_png.name, format="png", bbox_inches="tight")
    plt.close()
    return tmp_png.name