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

"""Shared fixtures for performance benchmarks."""

from knowledge_core.kb import KnowledgeCore
import pytest

from .generators import generate_flat_triples, generate_typed_triples


# ---------------------------------------------------------------------------
# KB size parameter
# ---------------------------------------------------------------------------

KB_SIZES = [100, 1_000, 10_000]

# For heavier benchmarks that would be too slow at 10K
KB_SIZES_LIGHT = [100, 1_000]


@pytest.fixture(params=[False, True], ids=['no_reasoner', 'reasoner'])
def reasoner_enabled(request):
    return request.param


@pytest.fixture(params=KB_SIZES, ids=[f'{n}_facts' for n in KB_SIZES])
def kb_size(request):
    return request.param


@pytest.fixture(params=KB_SIZES_LIGHT, ids=[f'{n}_facts' for n in KB_SIZES_LIGHT])
def kb_size_light(request):
    return request.param


# ---------------------------------------------------------------------------
# Fresh KB instances
# ---------------------------------------------------------------------------


@pytest.fixture
def kb_empty():
    """Fresh KnowledgeCore with no reasoner."""
    return KnowledgeCore(enable_reasoner=False)


@pytest.fixture
def kb_empty_reasoner():
    """Fresh KnowledgeCore with reasoner enabled."""
    return KnowledgeCore(enable_reasoner=True)


# ---------------------------------------------------------------------------
# Pre-populated KBs (flat triples, no ontological structure)
# ---------------------------------------------------------------------------

def _make_populated_kb(n, enable_reasoner=False):
    """Create a KB pre-loaded with n flat triples."""
    kb = KnowledgeCore(enable_reasoner=enable_reasoner)
    stmts = generate_flat_triples(n)
    # Load in batches to avoid massive single parse
    batch_size = 500
    for i in range(0, len(stmts), batch_size):
        kb.update(stmts[i:i + batch_size])
    return kb


@pytest.fixture
def kb_populated(kb_size):
    """KB with kb_size flat triples, no reasoner."""
    return _make_populated_kb(kb_size, enable_reasoner=False)


@pytest.fixture
def kb_populated_reasoner(kb_size_light):
    """KB with kb_size_light flat triples, reasoner enabled."""
    return _make_populated_kb(kb_size_light, enable_reasoner=True)


# ---------------------------------------------------------------------------
# Pre-populated KBs (typed triples, with classes)
# ---------------------------------------------------------------------------

def _make_typed_kb(n, enable_reasoner=False):
    """Create a KB pre-loaded with n typed instances (~3n triples)."""
    kb = KnowledgeCore(enable_reasoner=enable_reasoner)
    stmts = generate_typed_triples(n)
    batch_size = 500
    for i in range(0, len(stmts), batch_size):
        kb.update(stmts[i:i + batch_size])
    return kb


@pytest.fixture
def kb_typed(kb_size):
    """KB with typed instances, no reasoner."""
    return _make_typed_kb(kb_size, enable_reasoner=False)
