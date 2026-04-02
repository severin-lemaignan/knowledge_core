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

"""
Benchmarks isolating materialisation (reasoner) cost.

Separates: graph copy into reasoner, reasoning itself, graph copy back.
"""

from knowledge_core.kb import DEFAULT_MODEL, KnowledgeCore
import pytest
import rdflib
from rdflib import Graph

from .generators import generate_flat_triples, generate_hierarchy, generate_typed_triples

try:
    import reasonable
    HAS_REASONER = True
except ImportError:
    HAS_REASONER = False


# ---------------------------------------------------------------------------
# Full materialise() benchmarks
# ---------------------------------------------------------------------------


MATERIALISE_SIZES = [100, 1_000, 5_000]


class TestBenchMaterialise:

    @pytest.mark.parametrize(
        'n', MATERIALISE_SIZES, ids=[f'{n}' for n in MATERIALISE_SIZES]
    )
    def test_no_reasoner(self, benchmark, n):
        """materialise() with reasoner disabled (identity assignment)."""
        kb = KnowledgeCore(enable_reasoner=False)
        kb.update(generate_flat_triples(n))
        # Mark dirty to force re-materialisation
        kb.models[DEFAULT_MODEL].is_dirty = True
        benchmark(kb.materialise)

    @pytest.mark.skipif(not HAS_REASONER, reason='reasonable not installed')
    @pytest.mark.parametrize(
        'n', MATERIALISE_SIZES, ids=[f'{n}' for n in MATERIALISE_SIZES]
    )
    def test_with_reasoner_flat(self, benchmark, n):
        """materialise() with reasoner on flat (no-ontology) triples."""
        kb = KnowledgeCore(enable_reasoner=True)
        kb.update(generate_flat_triples(n))
        kb.models[DEFAULT_MODEL].is_dirty = True
        benchmark(kb.materialise)

    @pytest.mark.skipif(not HAS_REASONER, reason='reasonable not installed')
    @pytest.mark.parametrize(
        'n', MATERIALISE_SIZES, ids=[f'{n}' for n in MATERIALISE_SIZES]
    )
    def test_with_reasoner_typed(self, benchmark, n):
        """materialise() with reasoner on typed triples."""
        kb = KnowledgeCore(enable_reasoner=True)
        kb.update(generate_typed_triples(n))
        kb.models[DEFAULT_MODEL].is_dirty = True
        benchmark(kb.materialise)


# ---------------------------------------------------------------------------
# Hierarchy depth scaling
# ---------------------------------------------------------------------------


HIERARCHY_DEPTHS = [3, 5, 8]


class TestBenchMaterialiseHierarchy:

    @pytest.mark.skipif(not HAS_REASONER, reason='reasonable not installed')
    @pytest.mark.parametrize(
        'depth', HIERARCHY_DEPTHS,
        ids=[f'depth_{d}' for d in HIERARCHY_DEPTHS]
    )
    def test_hierarchy(self, benchmark, depth):
        """materialise() scaling with class hierarchy depth (breadth=2)."""
        kb = KnowledgeCore(enable_reasoner=True)
        stmts, _ = generate_hierarchy(depth, breadth=2)
        kb.update(stmts)
        kb.models[DEFAULT_MODEL].is_dirty = True
        benchmark(kb.materialise)


# ---------------------------------------------------------------------------
# Isolated reasoner steps (decomposing materialise)
# ---------------------------------------------------------------------------


@pytest.mark.skipif(not HAS_REASONER, reason='reasonable not installed')
class TestBenchReasonerSteps:
    """Benchmark individual steps of the materialisation pipeline."""

    @pytest.mark.parametrize(
        'n', MATERIALISE_SIZES, ids=[f'{n}' for n in MATERIALISE_SIZES]
    )
    def test_reasoner_from_graph(self, benchmark, n):
        """Time: PyReasoner().from_graph(g) -- copying triples into reasoner."""
        kb = KnowledgeCore(enable_reasoner=True)
        kb.update(generate_flat_triples(n))
        graph = kb.models[DEFAULT_MODEL].graph

        def from_graph():
            r = reasonable.PyReasoner()
            r.from_graph(graph)
            return r

        benchmark(from_graph)

    @pytest.mark.parametrize(
        'n', MATERIALISE_SIZES, ids=[f'{n}' for n in MATERIALISE_SIZES]
    )
    def test_reasoner_reason(self, benchmark, n):
        """Time: r.reason() -- the actual reasoning step."""
        kb = KnowledgeCore(enable_reasoner=True)
        kb.update(generate_flat_triples(n))
        graph = kb.models[DEFAULT_MODEL].graph

        r = reasonable.PyReasoner()
        r.from_graph(graph)

        # reason() returns triples; we need a fresh reasoner each time
        # since reason() may consume state
        def reason():
            r2 = reasonable.PyReasoner()
            r2.from_graph(graph)
            return r2.reason()

        benchmark(reason)

    @pytest.mark.parametrize(
        'n', MATERIALISE_SIZES, ids=[f'{n}' for n in MATERIALISE_SIZES]
    )
    def test_reasoner_update_graph(self, benchmark, n):
        """Time: r.update_graph(g) -- incremental diff + copy."""
        kb = KnowledgeCore(enable_reasoner=True)
        kb.update(generate_flat_triples(n))
        graph = kb.models[DEFAULT_MODEL].graph
        r = kb.models[DEFAULT_MODEL].reasoner

        benchmark(r.update_graph, graph)

    @pytest.mark.parametrize(
        'n', MATERIALISE_SIZES, ids=[f'{n}' for n in MATERIALISE_SIZES]
    )
    def test_reasoner_reason_incremental(self, benchmark, n):
        """Time: r.reason() after update_graph (incremental path)."""
        kb = KnowledgeCore(enable_reasoner=True)
        kb.update(generate_flat_triples(n))
        r = kb.models[DEFAULT_MODEL].reasoner
        graph = kb.models[DEFAULT_MODEL].graph

        counter = [0]

        def incremental_reason():
            i = counter[0]
            counter[0] += 1
            graph.add((
                rdflib.URIRef(f'http://kb.openrobots.org#incEnt{i}'),
                rdflib.URIRef('http://kb.openrobots.org#hasProp'),
                rdflib.URIRef(f'http://kb.openrobots.org#incObj{i}'),
            ))
            r.update_graph(graph)
            return r.reason()

        benchmark(incremental_reason)

    @pytest.mark.parametrize(
        'n', MATERIALISE_SIZES, ids=[f'{n}' for n in MATERIALISE_SIZES]
    )
    def test_reasoner_reason_full(self, benchmark, n):
        """Time: full from_graph + reason (comparison with incremental)."""
        kb = KnowledgeCore(enable_reasoner=True)
        kb.update(generate_flat_triples(n))
        graph = kb.models[DEFAULT_MODEL].graph

        def full_reason():
            r = reasonable.PyReasoner()
            r.from_graph(graph)
            return r.reason()

        benchmark(full_reason)

    @pytest.mark.parametrize(
        'n', MATERIALISE_SIZES, ids=[f'{n}' for n in MATERIALISE_SIZES]
    )
    def test_graph_copy_back(self, benchmark, n):
        """Time: Graph() += r.reason() -- copying inferred triples."""
        kb = KnowledgeCore(enable_reasoner=True)
        kb.update(generate_flat_triples(n))
        r = kb.models[DEFAULT_MODEL].reasoner

        reasoned = r.reason()

        def copy_back():
            g = Graph()
            g += reasoned
            return g

        benchmark(copy_back)
