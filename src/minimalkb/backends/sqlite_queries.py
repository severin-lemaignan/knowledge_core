import logging

logger = logging.getLogger("minimalKB." + __name__)
DEBUG_LEVEL = logging.DEBUG

from minimalkb.exceptions import KbServerError


def query(db, vars, patterns, models):
    """
    'vars' is the list of unbound variables that are expected to be returned.
    Each of them must start with a '?'.

    'patterns' is a list/set of 3-tuples (s,p,o). Each tuple may contain
    unbound variables, that MUST start with a '?'.
    """

    vars = set(vars)

    allvars = set()
    for p in patterns:
        allvars |= set(get_vars(p))

    if not allvars >= vars:
        logger.warn("Some requested vars are not present in the patterns. Returning []")
        return []

    if len(patterns) == 1:
        return singlepattern(db, patterns[0], models)

    independentpatterns = {p for p in patterns if nb_variables(p) == 1}
    dependentpatterns = set(patterns) - independentpatterns

    directpatterns = {}

    candidates = {}
    for v in allvars:
        directpatterns[v] = {p for p in patterns if v in p}

        # first, execute simple queries to determine potential candidates:
        # resolve patterns that contain *only* the desired output variable
        for p in independentpatterns & directpatterns[v]:
            if v not in candidates:
                candidates[v] = simplequery(db, p, models)
            else:
                # intersection with previous candidates
                candidates[v] = candidates[v] & simplequery(db, p, models)

    # if any of the requested var appears in an independant pattern but has no match for
    # this pattern, return []
    for var in allvars:
        if var in candidates and not candidates[var]:
            return []

    if len(vars) == 1:
        var = vars.pop()

        # no dependent pattern? no need to filter!
        if not dependentpatterns:
            return list(candidates[var])

        candidate = set()
        for pattern in dependentpatterns:
            if var not in pattern:
                raise NotImplementedError(
                    "Can not handle pattern %s with requested variable %s."
                    % (pattern, var)
                )

            def prepare(tok):
                if tok == var:
                    return None
                return candidates.get(tok, [tok])

            s, p, o = [prepare(tok) for tok in pattern]

            if not candidate:
                candidate = selectfromset(db, s, p, o, models)
            else:
                candidate &= selectfromset(db, s, p, o, models)
        return list(candidate)

    else:
        if not dependentpatterns:
            raise NotImplementedError(
                "Multiple variable in independent patterns not yet supported."
            )

        raise NotImplementedError(
            "Only a single variable in queries can be currently requested."
        )
        ### TODO !!! ###
        while dependentpatterns:
            pattern = dependentpatterns.pop()
            s, p, o = pattern
            stmts = [(r[1], r[2], r[3]) for r in matchingstmt(db, pattern, models)]

            if is_variable(s):
                pass


def singlepattern(db, pattern, models):
    """Returns the list of statements that match
    a single pattern (like "* likes ?toto").

    If only one unbound variable is present, it returns
    the list of possible values for this variable.

    If 2 or 3 tokens are unbound, it returns a list of
    complete statments (s,p,o).
    """
    if nb_variables(pattern) == 1:
        return list(simplequery(db, pattern, models))
    else:
        results = matchingstmt(db, pattern, models)
        return [[res[1], res[2], res[3]] for res in results]


def get_vars(s):
    return [x for x in s if x.startswith("?")]


def nb_variables(s):
    return len(get_vars(s))


def is_variable(tok):
    return tok and tok.startswith("?")


def matchingstmt(db, pattern, models=[], assertedonly=False):
    """Returns the list of statements matching a given pattern.

    If assertedonly is True, statements infered by reasoning are
    excluded.
    """

    s, p, o = pattern
    params = {
        "s": s,
        "p": p,
        "o": o,
    }

    # workaround to feed a variable number of models
    models = list(models)
    for i in range(len(models)):
        params["m%s" % i] = models[i]

    query = "SELECT * FROM triples "
    conditions = []
    if not is_variable(s):
        conditions += ["subject=:s"]
    if not is_variable(p):
        conditions += ["predicate=:p"]
    if not is_variable(o):
        conditions += ["object=:o"]

    if assertedonly:
        conditions += ["inferred=0"]
    if models:
        conditions += [
            "model IN (%s)" % (",".join([":m%s" % i for i in range(len(models))]))
        ]

    if conditions:
        query += "WHERE (" + " AND ".join(conditions) + ")"

    return [row for row in db.execute(query, params)]


def selectfromset(
    db, subject=None, predicate=None, object=None, models=[], assertedonly=False
):

    if (
        (not subject and not predicate)
        or (not subject and not object)
        or (not predicate and not object)
        or (subject and predicate and object)
    ):
        import pdb

        pdb.set_trace()
        raise KbServerError("Exactly one of subject, predicate or object must be None")
    params = {}

    # workaround to feed a variable number of models
    models = list(models)
    for i in range(len(models)):
        params["m%s" % i] = models[i]

    selectedcolumn = (
        "subject" if not subject else ("predicate" if not predicate else "object")
    )

    query = "SELECT %s FROM triples " % selectedcolumn

    conditions = []
    if subject:
        conditions += ["subject IN ('" + "','".join(subject) + "')"]
    if predicate:
        conditions += ["predicate IN ('" + "','".join(predicate) + "')"]
    if object:
        conditions += ["object IN ('" + "','".join(object) + "')"]

    if assertedonly:
        conditions += ["inferred=0"]
    if models:
        conditions += [
            "model IN (%s)" % (",".join([":m%s" % i for i in range(len(models))]))
        ]

    if conditions:
        query += "WHERE (" + " AND ".join(conditions) + ")"

    return {row[0] for row in db.execute(query, params)}


def simplequery(db, pattern, models=[], assertedonly=False):
    """A 'simple query' is a query with only *one* unbound variable.

    Return the list of possible values for this variable
    """

    s, p, o = pattern
    params = {
        "s": s,
        "p": p,
        "o": o,
    }

    # workaround to feed a variable number of models
    models = list(models)
    for i in range(len(models)):
        params["m%s" % i] = models[i]

    query = "SELECT "
    if is_variable(s):
        query += "subject FROM triples WHERE (predicate=:p AND object=:o)"
    elif is_variable(p):
        query += "predicate FROM triples WHERE (subject=:s AND object=:o)"
    elif is_variable(o):
        query += "object FROM triples WHERE (subject=:s AND predicate=:p)"
    else:
        query += "hash FROM triples WHERE (subject=:s AND predicate=:p AND object=:o)"

    if assertedonly:
        query += " AND inferred=0"
    if models:
        query += " AND model IN (%s)" % (
            ",".join([":m%s" % i for i in range(len(models))])
        )

    return {row[0] for row in db.execute(query, params)}
