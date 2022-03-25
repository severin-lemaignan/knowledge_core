BACKENDS = []

SQLITE = "sqlite"
BACKENDS.append(SQLITE)
DEFAULT_BACKEND = SQLITE

RDFLIB = "rdflib"
try:
    import rdflib
    import rdflib.namespace

    BACKENDS.append(RDFLIB)
    DEFAULT_BACKEND = RDFLIB
except ImportError:
    pass
