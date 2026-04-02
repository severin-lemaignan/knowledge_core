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
End-to-end pipeline benchmarks.

Measures the full cost of update() including parsing, graph operations,
materialisation, and event evaluation -- the real world latency a
robotics application experiences.
"""

import itertools

from knowledge_core.kb import KnowledgeCore
import pytest

from .generators import generate_typed_triples


PIPELINE_SIZES = [100, 1_000, 5_000]


def _make_pipeline_kb(n, enable_reasoner, n_events=0):
    kb = KnowledgeCore(enable_reasoner=enable_reasoner)
    kb.update(generate_typed_triples(n))

    for i in range(n_events):
        evt_id = kb.subscribe([f'?x rdf:type PipeEvtClass{i}'])
        kb.eventsubscriptions.setdefault(evt_id, []).append(
            f'pipe_client_{i}'
        )

    return kb


class TestBenchPipeline:
    """Full update pipeline: parse -> graph.add -> materialise -> events."""

    @pytest.mark.parametrize(
        'n', PIPELINE_SIZES, ids=[f'{n}' for n in PIPELINE_SIZES]
    )
    def test_single_update_no_reasoner(self, benchmark, n):
        kb = _make_pipeline_kb(n, enable_reasoner=False)
        counter = itertools.count()

        def pipeline():
            i = next(counter)
            kb.update([f'pipeEnt{i} rdf:type BenchClass0'])

        benchmark(pipeline)

    @pytest.mark.parametrize(
        'n', PIPELINE_SIZES, ids=[f'{n}' for n in PIPELINE_SIZES]
    )
    def test_single_update_with_reasoner(self, benchmark, n):
        kb = _make_pipeline_kb(n, enable_reasoner=True)
        counter = itertools.count()

        def pipeline():
            i = next(counter)
            kb.update([f'pipeReasEnt{i} rdf:type BenchClass0'])

        benchmark(pipeline)

    @pytest.mark.parametrize(
        'n', PIPELINE_SIZES, ids=[f'{n}' for n in PIPELINE_SIZES]
    )
    def test_single_update_with_events(self, benchmark, n):
        kb = _make_pipeline_kb(n, enable_reasoner=False, n_events=10)
        counter = itertools.count()

        def pipeline():
            i = next(counter)
            kb.update([f'pipeEvtEnt{i} hasProp pipeEvtObj{i}'])

        benchmark(pipeline)

    @pytest.mark.parametrize(
        'n', PIPELINE_SIZES, ids=[f'{n}' for n in PIPELINE_SIZES]
    )
    def test_single_update_full_stack(self, benchmark, n):
        """Reasoner + 10 events -- worst case."""
        kb = _make_pipeline_kb(n, enable_reasoner=True, n_events=10)
        counter = itertools.count()

        def pipeline():
            i = next(counter)
            kb.update([f'fullEnt{i} rdf:type BenchClass0'])

        benchmark(pipeline)


class TestBenchPipelineQueryAfterUpdate:
    """Measure update-then-query cycle (the typical robotics pattern)."""

    @pytest.mark.parametrize(
        'n', PIPELINE_SIZES, ids=[f'{n}' for n in PIPELINE_SIZES]
    )
    def test_update_then_find(self, benchmark, n):
        kb = _make_pipeline_kb(n, enable_reasoner=False)
        counter = itertools.count()

        def update_and_query():
            i = next(counter)
            kb.update([f'uqEnt{i} rdf:type BenchClass0'])
            return kb.find(['?x rdf:type BenchClass0'], ['?x'])

        result = benchmark(update_and_query)
        assert len(result) > 0


class TestBenchBatchMode:
    """Benchmark the batch() context manager vs sequential updates."""

    @pytest.mark.parametrize(
        'n', PIPELINE_SIZES, ids=[f'{n}' for n in PIPELINE_SIZES]
    )
    def test_10_updates_sequential(self, benchmark, n):
        """10 sequential update() calls (no batch)."""
        kb = _make_pipeline_kb(n, enable_reasoner=True)
        counter = itertools.count()

        def sequential():
            base = next(counter) * 10
            for j in range(10):
                kb.update([f'seqEnt{base + j} rdf:type BenchClass0'])

        benchmark(sequential)

    @pytest.mark.parametrize(
        'n', PIPELINE_SIZES, ids=[f'{n}' for n in PIPELINE_SIZES]
    )
    def test_10_updates_batched(self, benchmark, n):
        """10 update() calls inside a batch() context."""
        kb = _make_pipeline_kb(n, enable_reasoner=True)
        counter = itertools.count()

        def batched():
            base = next(counter) * 10
            with kb.batch():
                for j in range(10):
                    kb.update(
                        [f'batEnt{base + j} rdf:type BenchClass0']
                    )

        benchmark(batched)
