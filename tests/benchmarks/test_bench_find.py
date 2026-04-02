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
Benchmarks for fact retrieval (find/exist/sparql).

Parameterised by KB size to show scaling behaviour.
"""


# ---------------------------------------------------------------------------
# find() benchmarks
# ---------------------------------------------------------------------------


class TestBenchFind:

    def test_single_pattern_selective(self, benchmark, kb_typed):
        """Query matching a single specific class (few results)."""
        result = benchmark(
            kb_typed.find,
            ['?x rdf:type BenchClass0'],
            ['?x'],
        )
        assert len(result) > 0

    def test_single_pattern_broad(self, benchmark, kb_typed):
        """Query with a broad wildcard predicate."""
        result = benchmark(
            kb_typed.find,
            ['?x hasName ?name'],
            ['?x', '?name'],
        )
        assert len(result) > 0

    def test_two_pattern_join(self, benchmark, kb_typed):
        """Two-pattern join: type + property."""
        result = benchmark(
            kb_typed.find,
            ['?x rdf:type BenchClass0', '?x hasName ?name'],
            ['?x', '?name'],
        )
        assert len(result) > 0

    def test_three_pattern_join(self, benchmark, kb_typed):
        """Three-pattern join."""
        result = benchmark(
            kb_typed.find,
            [
                '?x rdf:type BenchClass0',
                '?x hasName ?name',
                '?x hasIndex ?idx',
            ],
            ['?x', '?name', '?idx'],
        )
        assert len(result) > 0

    def test_no_results(self, benchmark, kb_typed):
        """Query that matches nothing."""
        result = benchmark(
            kb_typed.find,
            ['?x rdf:type NonExistentClass'],
            ['?x'],
        )
        assert len(result) == 0


# ---------------------------------------------------------------------------
# exist() benchmarks
# ---------------------------------------------------------------------------


class TestBenchExist:

    def test_exist_true(self, benchmark, kb_typed):
        """Check existence of a known fact."""
        result = benchmark(
            kb_typed.exist, ['benchInst0 rdf:type BenchClass0']
        )
        assert result is True

    def test_exist_false(self, benchmark, kb_typed):
        """Check existence of a non-existent fact."""
        result = benchmark(
            kb_typed.exist, ['nonExistent rdf:type BenchClass0']
        )
        assert result is False


# ---------------------------------------------------------------------------
# sparql() benchmarks
# ---------------------------------------------------------------------------


class TestBenchSparql:

    def test_simple_select(self, benchmark, kb_typed):
        """Direct SPARQL SELECT query."""
        result = benchmark(
            kb_typed.sparql,
            'SELECT ?x WHERE { ?x rdf:type :BenchClass0 }',
        )
        assert len(result) > 0

    def test_count_query(self, benchmark, kb_typed):
        """SPARQL COUNT query."""
        result = benchmark(
            kb_typed.sparql,
            'SELECT (COUNT(?x) AS ?count) WHERE { ?x ?p ?o }',
        )
        assert len(result) > 0
