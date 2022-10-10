import sys
import math
import xml.etree.ElementTree as ET

"""
This script converts an OpenStreetMap XML file to a graph
"""

SCALE_MUL = 1000

WEIGHTS = {
	"motorway": 1,
	"trunk": 2,
	"primary": 3,
	"secondary": 4,
	"tertiary": 5,
	"unclassified": 5,
	"residential": 5
}

if len(sys.argv) < 2:
	print("usage:", sys.argv[0], "input_file")


tree = ET.parse(sys.argv[1])
root = tree.getroot()

nodes = {}
graphNodes = {}
graphEdges = []

def process_road(r, hw):
	nd = [ch.attrib["ref"] for ch in r if ch.tag == "nd"]
	if len(nd) < 2:
		print("[WARN] road has less than 2 nodes")
		return
	for n0, n1 in zip(nd[1:], nd):
		if n0 not in nodes or n1 not in nodes:
			print("[ERR] node not found")
			continue
		xy0 = (nodes[n0].attrib["lat"], nodes[n0].attrib["lon"])
		xy1 = (nodes[n1].attrib["lat"], nodes[n1].attrib["lon"])
		xy0 = tuple(map(lambda x: float(x) * SCALE_MUL, xy0))
		xy1 = tuple(map(lambda x: float(x) * SCALE_MUL, xy1))
		graphNodes[n0] = { "x": xy0[0], "y": xy0[1] }
		graphNodes[n1] = { "x": xy1[0], "y": xy1[1] }
		weight = math.sqrt((float(xy0[0]) - float(xy1[0])) ** 2 + (float(xy0[1]) - float(xy1[1])) ** 2)
		weight *= WEIGHTS[hw]
		graphEdges.append({ "n0": n0, "n1": n1, "weight": weight})

for child in root:
	if child.tag == "node":
		nodes[child.attrib["id"]] = child
	elif child.tag == "way":
		for child2 in child:
			if child2.tag == "tag" and child2.attrib["k"] == "highway" and child2.attrib["v"] in WEIGHTS:
				process_road(child, child2.attrib["v"])

filename = sys.argv[1].split(".")[0] + "_" + "-".join(map(str, WEIGHTS.values())) + ".txt"
f = open(filename, "w")

f.write(str(len(graphNodes)) + "\n")

print(f"created graph with {len(graphNodes)} nodes and {len(graphEdges)} edges, writing to file...")

i = 0
for v in graphNodes.values():
	f.write(f"""{v["x"]:.3f} {v["y"]:.3f}\n""")
	v["id"] = i
	i += 1

for e in graphEdges:
	n0 = graphNodes[e["n0"]]["id"]
	n1 = graphNodes[e["n1"]]["id"]
	f.write(f"""{n0} {n1} {e["weight"]:.3f}\n""")

print(f"wrote graph with {len(graphNodes)} nodes and {len(graphEdges)} edges")
