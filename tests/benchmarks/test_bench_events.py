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
Benchmarks for the event subscription/evaluation system.

Measures overhead added by active events on every update.
"""

import itertools

from knowledge_core.kb import KnowledgeCore
import pytest

from .generators import generate_flat_triples


EVENT_COUNTS = [0, 1, 5, 20]

KB_PRELOAD = 1_000


def _make_kb_with_events(n_events, enable_reasoner=False):
    """Create a KB with n_events active subscriptions and a dummy client."""
    kb = KnowledgeCore(enable_reasoner=enable_reasoner)
    kb.update(generate_flat_triples(KB_PRELOAD))

    for i in range(n_events):
        evt_id = kb.subscribe([f'?x rdf:type EvtTriggerClass{i}'])
        # Register a fake client for the event so it stays active
        kb.eventsubscriptions.setdefault(evt_id, []).append(
            f'bench_client_{i}'
        )

    return kb


class TestBenchUpdateWithEvents:
    """Measure update() cost as number of active events increases."""

    @pytest.mark.parametrize(
        'n_events', EVENT_COUNTS,
        ids=[f'{n}_events' for n in EVENT_COUNTS]
    )
    def test_update_no_trigger(self, benchmark, n_events):
        """Update with a fact that does NOT trigger any event."""
        kb = _make_kb_with_events(n_events)
        counter = itertools.count()

        def update():
            i = next(counter)
            kb.update([f'evtBenchEntity{i} hasEvtProp evtBenchObj{i}'])

        benchmark(update)

    @pytest.mark.parametrize(
        'n_events', [1, 5, 20],
        ids=['1_events', '5_events', '20_events']
    )
    def test_update_triggers_one_event(self, benchmark, n_events):
        """Update that triggers the first event (adds matching type)."""
        kb = _make_kb_with_events(n_events)
        counter = itertools.count()

        def update():
            i = next(counter)
            kb.update([f'triggerInst{i} rdf:type EvtTriggerClass0'])

        benchmark(update)


class TestBenchSubscribe:
    """Measure the cost of subscribing to a new event."""

    def test_subscribe_simple(self, benchmark, kb_empty):
        counter = itertools.count()

        def subscribe():
            i = next(counter)
            return kb_empty.subscribe([f'?x rdf:type SubClass{i}'])

        result = benchmark(subscribe)
        assert result.startswith('evt_')

    def test_subscribe_complex(self, benchmark, kb_empty):
        counter = itertools.count()

        def subscribe():
            i = next(counter)
            return kb_empty.subscribe(
                [
                    f'?x rdf:type SubClass{i}',
                    '?x hasStatus "active"',
                    '?x hasOwner ?owner',
                ]
            )

        result = benchmark(subscribe)
        assert result.startswith('evt_')
