minimalkb API
=============

Core API
--------

### Models

Several methods take an optional `models` parameter. If set to `None` or to an
empty list, the method will update/query statements in the robot's base
cognitive model. If set to a list containing a single string `all`, all the
existing models are update/queried. Otherwise, you can pass a list of modelsyou
want to update/query.

### Methods

- `about(resource, models=None)`
- `add(stmts, models=None, lifespan=0)`: adds statements to the given model(s)
  with the given lifespan. Alias for `revise` with `policy['method']='add'`.
- `check(stmts, models=None)`: 
- `classesof(concept, direct=False, models=None)`
- `clear()`
- `details(resource, models=None)`: returns a dictionary containing the
  following details on a given resource:
  - `name`: resource label, if any, literal value for literals, else resource ID.
  - `id`: resource ID or `literal` for literals
  - `type`: one of `['instance', 'class', 'object_property', 'datatype_property', 'undecided']`
  - `sameAs`: list of equivalent classes or instances, if any
  - 'attributes':
    -  for classes, a list of three dictionaries:
       `{"name": "Parents","id": "superClasses", "values":[ids...]}`
       `{"name": "Children","id": "subClasses", "values":[ids...]}`
       `{"name": "Instances","id": "instances", "values":[ids...]}`
        (only direct super/sub-classes and instances)
    -  for instances, a list of one dictionary:
       `{"name": "Classes","id": "classes", "values":[ids...]}`
       (only direct classes)

- `exist(stmts, models=None)`
- `find(vars, patterns, constraints=None, models=None)`: depending on the
  arguments, three differents behaviours are possible:
  - if `len(vars) == 1`, `find` returns a set of resources matching the patterns.
  - if `len(vars) > 1`:
    - if `len(patterns) == 1`, a list of statements matching the pattern
      is returned.
    - else, a list of dictionaries is returned with
    possible combination of values for the different variables. For
    instance, `find(["?agent", "?action"], ["?agent desires ?action", "?action rdf:type Jump"])`
    would return something like: `[{"agent":"james", "action": "jumpHigh"}, {"agent": "laurel", "action":"jumpHigher"}]`

  Note that `constraints` is currently not supported.
- `findmpe(vars, pattern, constraints=None, models=None)`: finds the most
  probable explanation. Currently strictly equivalent to `find`, as `minimalkb`
  does not currently support probabilistic reasoning.
- `hello()`: returns the version number of the `minimalkb` server as a string. Can be used
  to check connection status.
- `load(filename, models=None)`: loads the content of the specified OWL ontology. Note that
  this capability is only available if
  [RDFlib](https://rdflib.readthedocs.io/en/stable/) is available to the server.
- `lookup(resource, models=None)`
- `methods()`: list available methods
- `retract(stmts, models=None)`: alias for `revise` with
  `policy['method']='retract'`.
- `revise(stmts, policy)`: Add/retract/updates one or several statements in the
  specified model.

  `policy` is a dictionary with the following fields:
  -  `method` (required): string in [`add`, `safe_add`, `retract`, `update`, `safe_update`, `revision`]
  -  `models` (optional, default to `None`): list of strings
  -  `lifespan` (optional, default to `0`): duration before automatic removal of statements, in
     seconds, float. If set to 0, statements do not expire.


- `subscribe(type, trigger, var, patterns, models=None)`
- `update(stmts, models=None, lifespan=0)`: updates statements in the given model(s)
  with the given lifespan. Alias for `revise` with `policy['method']='update'`.
  If the predicate(s) are *not* inferred to be functional (i.e., it accept only
  one single value), behaves like `add`.

Backward compatibility with ORO
-------------------------------

*Do not use these function calls in new code. They might be removed in the
future.*

- `findForAgent(agent, var, stmts) (compatibility)`
- `getClassesOf(concept, direct=False) (compatibility)``
- `getDirectClassesOf(concept) (compatibility)`
- `getLabel(concept) (compatibility)`
- `getResourceDetails(concept) (compatibility)`
- `listAgents() (compatibility)`
- `listSimpleMethods() (compatibility)`
- `lookupForAgent(agent, resource) (compatibility)`
- `registerEvent(type, trigger, patterns) (compatibility)`
- `remove(stmts, models=None) (compatibility)`
- `removeForAgent(agent, stmts) (compatibility)`
- `reset() (compatibility)`
- `safeAdd(stmts, lifespan=0) (compatibility)`
- `stats() (compatibility)`

