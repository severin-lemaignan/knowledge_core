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

import functools


def memoize(maxsize=4096):
    """LRU-bounded cache for functions with hashable arguments.

    Usage::

        @memoize()
        def parse_stmt(stmt): ...

        @memoize(maxsize=1024)
        def parse_term(term): ...

    For functions that accept lists, the caller must convert to tuple first
    (see ``memoize_list_args``).

    Results equal to ``'undecided'`` are never cached -- they must be
    re-evaluated every time.
    """

    def decorator(fn):
        cache = {}
        hits = misses = 0
        eviction_order = []  # simple FIFO for eviction

        @functools.wraps(fn)
        def wrapper(*args, **kwargs):
            nonlocal hits, misses
            key = (args, tuple(sorted(kwargs.items()))) if kwargs else args
            if key in cache:
                hits += 1
                return cache[key]

            res = fn(*args, **kwargs)

            # never memoize undecided reasoning result
            if res == 'undecided':
                return res

            misses += 1
            if len(cache) >= maxsize:
                # evict oldest entry
                while eviction_order:
                    old_key = eviction_order.pop(0)
                    if old_key in cache:
                        del cache[old_key]
                        break

            cache[key] = res
            eviction_order.append(key)
            return res

        wrapper.cache = cache
        wrapper.cache_clear = lambda: (cache.clear(), eviction_order.clear())
        wrapper.cache_info = lambda: {'hits': hits, 'misses': misses, 'size': len(cache)}
        return wrapper

    return decorator


def memoize_list_args(maxsize=4096):
    """Like ``memoize`` but converts the first list argument to a tuple for hashing."""

    def decorator(fn):
        inner = memoize(maxsize=maxsize)(fn)
        original_fn = fn

        @functools.wraps(fn)
        def wrapper(stmts, *args, **kwargs):
            hashable_stmts = tuple(stmts) if isinstance(stmts, list) else stmts
            return inner(hashable_stmts, *args, **kwargs)

        wrapper.cache = inner.cache
        wrapper.cache_clear = inner.cache_clear
        wrapper.cache_info = inner.cache_info
        return wrapper

    return decorator
