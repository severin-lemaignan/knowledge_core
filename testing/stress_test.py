import concurrent.futures
from kb import KB
import time

rounds = 100

hammer_idx = 0


def hammer(idx):

    kb = KB()

    i = 0

    while i < rounds:
        kb += [
            f"s_{idx}_{i} rdf:type C_{idx}_{i}",
            f"C_{idx}_{i} owl:equivalentClass C2_{idx}_{i}",
        ]
        res = kb[f"* rdf:type C2_{idx}_{i}"]
        assert len(res) == 1 and res[0] == f"s_{idx}_{i}"
        i += 1

    kb.close()


start = time.time()

NB_PARALLEL_CLIENTS = 10

with concurrent.futures.ThreadPoolExecutor(max_workers=NB_PARALLEL_CLIENTS) as executor:
    future_hammers = [
        executor.submit(hammer, idx) for idx in range(NB_PARALLEL_CLIENTS)
    ]

    concurrent.futures.wait(future_hammers)

duration = time.time() - start
print(f"Done. Execution time: {duration:.2f}s")
