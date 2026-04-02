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
Benchmarks for fact revision (add/update/remove).

Parameterised by KB size to show how performance scales.
"""

import itertools


# ---------------------------------------------------------------------------
# Update (add) benchmarks
# ---------------------------------------------------------------------------


class TestBenchUpdate:
    """Benchmark kb.update() at various KB sizes."""

    def _make_counter(self):
        return itertools.count()

    def test_single_fact(self, benchmark, kb_populated):
        counter = self._make_counter()

        def add_one():
            i = next(counter)
            kb_populated.update([f'newEntity{i} hasNewProp newObj{i}'])

        benchmark(add_one)

    def test_batch_10(self, benchmark, kb_populated):
        counter = self._make_counter()

        def add_batch():
            base = next(counter) * 10
            stmts = [
                f'batchEnt{base + j} hasBatchProp batchObj{base + j}'
                for j in range(10)
            ]
            kb_populated.update(stmts)

        benchmark(add_batch)

    def test_batch_100(self, benchmark, kb_populated):
        counter = self._make_counter()

        def add_batch():
            base = next(counter) * 100
            stmts = [
                f'bigBatchEnt{base + j} hasProp bigBatchObj{base + j}'
                for j in range(100)
            ]
            kb_populated.update(stmts)

        benchmark(add_batch)


# ---------------------------------------------------------------------------
# Update with reasoner
# ---------------------------------------------------------------------------


class TestBenchUpdateWithReasoner:
    """Same as above but with reasoner enabled."""

    def _make_counter(self):
        return itertools.count()

    def test_single_fact(self, benchmark, kb_populated_reasoner):
        counter = self._make_counter()

        def add_one():
            i = next(counter)
            kb_populated_reasoner.update(
                [f'rNewEntity{i} hasNewProp rNewObj{i}']
            )

        benchmark(add_one)

    def test_batch_10(self, benchmark, kb_populated_reasoner):
        counter = self._make_counter()

        def add_batch():
            base = next(counter) * 10
            stmts = [
                f'rBatchEnt{base + j} hasProp rBatchObj{base + j}'
                for j in range(10)
            ]
            kb_populated_reasoner.update(stmts)

        benchmark(add_batch)


# ---------------------------------------------------------------------------
# Remove benchmarks
# ---------------------------------------------------------------------------


class TestBenchRemove:
    """Benchmark kb.remove() at various KB sizes."""

    def test_single_fact(self, benchmark, kb_populated):
        # Pre-add facts to remove (one per benchmark round)
        facts = [f'rmEntity{i} hasRmProp rmObj{i}' for i in range(1000)]
        kb_populated.update(facts)
        counter = itertools.count()

        def remove_one():
            i = next(counter) % 1000
            kb_populated.remove([f'rmEntity{i} hasRmProp rmObj{i}'])

        benchmark(remove_one)

    def test_wildcard_remove(self, benchmark, kb_populated):
        # Pre-add facts with a common predicate
        facts = [f'wcEntity{i} hasWcProp wcObj{i}' for i in range(500)]
        kb_populated.update(facts)
        counter = itertools.count()

        def remove_wildcard():
            i = next(counter) % 500
            kb_populated.remove([f'wcEntity{i} ?p ?o'])

        benchmark(remove_wildcard)
