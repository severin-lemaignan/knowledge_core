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

"""Benchmarks isolating N3 parsing overhead.

These measure the cost of parse_stmt(), parse_stmts_to_graph(), and
parse_term() independently from any KB operations.
"""

import pytest

from knowledge_core.kb import parse_stmt, parse_stmts_to_graph, parse_term


# ---------------------------------------------------------------------------
# parse_stmt
# ---------------------------------------------------------------------------


class TestBenchParseStmt:

    def test_simple_triple(self, benchmark):
        result = benchmark(parse_stmt, "alice rdf:type Human")
        assert len(result) == 3

    def test_with_literal(self, benchmark):
        result = benchmark(parse_stmt, 'alice hasName "Alice"')
        assert len(result) == 3

    def test_with_typed_literal(self, benchmark):
        result = benchmark(parse_stmt, 'alice hasAge "30"^^xsd:integer')
        assert len(result) == 3

    def test_with_uri(self, benchmark):
        result = benchmark(parse_stmt, "alice owl:sameAs bob")
        assert len(result) == 3

    def test_long_names(self, benchmark):
        result = benchmark(
            parse_stmt,
            "veryLongEntityNameForBenchmark hasVeryLongProperty anotherLongObjectName",
        )
        assert len(result) == 3


# ---------------------------------------------------------------------------
# parse_stmts_to_graph (batch)
# ---------------------------------------------------------------------------


BATCH_SIZES = [1, 5, 10, 50]


class TestBenchParseStmtsToGraph:

    @pytest.mark.parametrize("n", BATCH_SIZES, ids=[f"batch_{n}" for n in BATCH_SIZES])
    def test_batch(self, benchmark, n):
        stmts = [f"entity{i} hasProp obj{i}" for i in range(n)]
        g = benchmark(parse_stmts_to_graph, stmts)
        assert len(list(g)) == n


# ---------------------------------------------------------------------------
# parse_term
# ---------------------------------------------------------------------------


class TestBenchParseTerm:

    def test_simple_name(self, benchmark):
        result = benchmark(parse_term, "alice")
        assert result is not None

    def test_prefixed_name(self, benchmark):
        result = benchmark(parse_term, "rdf:type")
        assert result is not None

    def test_string_literal(self, benchmark):
        result = benchmark(parse_term, '"hello world"')
        assert result is not None

    def test_typed_literal(self, benchmark):
        result = benchmark(parse_term, '"42"^^xsd:integer')
        assert result is not None
