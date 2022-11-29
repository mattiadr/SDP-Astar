import sys
import math
from lxml import etree
from dataclasses import dataclass

"""
This script converts an OpenStreetMap XML file to a graph
"""

SCALE_MUL = 1000

WEIGHTS = {
	"motorway":     1,
	"trunk":        2,
	"primary":      3,
	"secondary":    4,
	"tertiary":     5,
	"unclassified": 5,
	"residential":  5
}


@dataclass
class Node:
	x: float
	y: float
	id: int = 0


@dataclass
class Edge:
	n0: int
	n1: int
	weight: float


def process_road(r, hw):
	nd = [ch.attrib["ref"] for ch in r if ch.tag == "nd"]
	if len(nd) < 2:
		# print("[WARN] road has less than 2 nodes")
		return
	for n0, n1 in zip(nd[1:], nd):
		if n0 not in nodes or n1 not in nodes:
			print("[ERR] node not found")
			continue
		node0 = nodes[n0]
		node1 = nodes[n1]
		weight = math.sqrt((float(node0.x) - float(node1.x)) ** 2 + (float(node0.y) - float(node1.y)) ** 2)
		weight *= WEIGHTS[hw]
		edges.append(Edge(n0, n1, weight))
		node0.id = 1
		node1.id = 1


if len(sys.argv) < 2:
	print("usage:", sys.argv[0], "input_file")

nodes = {}
edges = []

for event, child in etree.iterparse(sys.argv[1], events=("end",)):
	if child.tag == "node":
		node_id = child.attrib["id"]
		if node_id not in nodes:
			x = float(child.attrib["lon"]) * SCALE_MUL
			y = float(child.attrib["lat"]) * SCALE_MUL
			nodes[node_id] = Node(x, y)
	elif child.tag == "way":
		for child2 in list(child):
			if child2.tag == "tag" and child2.attrib["k"] == "highway" and child2.attrib["v"] in WEIGHTS:
				process_road(child, child2.attrib["v"])

num_nodes = sum(n.id for n in nodes.values())

print(f"total nodes in xml {len(nodes)}")
print(f"created graph with {num_nodes} nodes and {len(edges)} edges, writing to file...")

filename = sys.argv[1].split(".")[0] + "_" + "-".join(map(str, WEIGHTS.values())) + ".txt"
f = open(filename, "w")

f.write(str(num_nodes) + "\n")

i = 0
for v in nodes.values():
	if v.id == 1:
		f.write(f"""{v.x:.3f} {v.y:.3f}\n""")
		v.id = i
		i += 1

for e in edges:
	n0 = nodes[e.n0].id
	n1 = nodes[e.n1].id
	f.write(f"""{n0} {n1} {e.weight:.3f}\n""")

print(f"wrote graph with {num_nodes} nodes and {len(edges)} edges")
