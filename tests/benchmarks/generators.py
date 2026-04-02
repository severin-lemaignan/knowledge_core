# Copyright 2026 IIIA-CSIC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Synthetic triple generators for performance benchmarking."""


def generate_flat_triples(n):
    """Generate n simple triples with no ontological structure.

    Each triple: entityN hasPropertyM valueK
    Uses rotating predicates to avoid a single-predicate index hotspot.
    """
    predicates = ["hasValue", "hasLabel", "hasScore", "hasTag", "hasNote"]
    stmts = []
    for i in range(n):
        p = predicates[i % len(predicates)]
        stmts.append(f"benchEntity{i} {p} benchObj{i}")
    return stmts


def generate_typed_triples(n, num_classes=10):
    """Generate n instances distributed across num_classes classes.

    Each instance gets rdf:type + 2 additional properties.
    Returns ~3*n statements total.
    """
    stmts = []
    for i in range(n):
        cls = f"BenchClass{i % num_classes}"
        stmts.append(f"benchInst{i} rdf:type {cls}")
        stmts.append(f"benchInst{i} hasName \"instance{i}\"")
        stmts.append(f"benchInst{i} hasIndex \"{i}\"^^xsd:integer")
    return stmts


def generate_hierarchy(depth, breadth):
    """Generate a class hierarchy with instances at the leaves.

    Creates a tree of rdfs:subClassOf relationships:
    - Root: BenchRoot
    - Each level has `breadth` children per parent
    - Instances attached at leaf classes

    Returns (stmts, num_instances) tuple.
    """
    stmts = []
    class_id = 0
    current_level = ["BenchRoot"]

    for d in range(depth):
        next_level = []
        for parent in current_level:
            for b in range(breadth):
                child = f"BenchCls_{class_id}"
                class_id += 1
                stmts.append(f"{child} rdfs:subClassOf {parent}")
                next_level.append(child)
        current_level = next_level

    # Add instances at leaves
    num_instances = 0
    for leaf in current_level:
        inst = f"benchLeaf{num_instances}"
        stmts.append(f"{inst} rdf:type {leaf}")
        num_instances += 1

    return stmts, num_instances


def generate_event_patterns(n):
    """Generate n distinct event subscription patterns of varying complexity."""
    patterns = []
    for i in range(n):
        if i % 3 == 0:
            # Single pattern
            patterns.append([f"?x rdf:type BenchEvtClass{i}"])
        elif i % 3 == 1:
            # Two-pattern join
            patterns.append(
                [f"?x rdf:type BenchEvtClass{i}", f"?x hasStatus \"active\""]
            )
        else:
            # Three-pattern join
            patterns.append(
                [
                    f"?x rdf:type BenchEvtClass{i}",
                    f"?x hasStatus \"active\"",
                    f"?x hasOwner ?owner",
                ]
            )
    return patterns
