minimalkb API
=============

Core API
--------

### Statements

`minimalkb` currently only support binary predicates, using the infix syntax to
represent symbolic statements as triples: `subject predicate object`.
For instance: `sky0 hasColor blue`

Triples *must* follow the [Turtle
syntax](https://www.w3.org/TR/turtle/#language-features): 
statements are represented as a single string, with the subject, predicate and
object separated by a space character.

In a nutshell:

- subjects and predicates must by valid RDF identifiers;
- objects can be either RDF identifiers, or literals. String literals must be
  surrounded by double quotes.
- RDF identifiers might include XML namespaces. Prefixes can be used (see below
  the list of recognised prefixes), separated with a semi colon. For instance:
  `james rdf:type oro:Human`
- as the triples are actually parsed with the
  [N3](https://www.w3.org/TeamSubmission/n3/) grammar (a super-set of Turtle),
  the namespace/prefix can actually be omitted altogether. In that case, the
  default prefix of the OpenRobots Ontology (`oro:`) is used.
- the means that all the terms defined in the OpenRobots ontology (eg `Robot`) can be used
  without prefix, and all new terms added without a specific prefix will be
  added to the OpenRobots ontology namespace.

Triples might occasionaly also include *variables* (eg, unbound terms).
Variables must start with a question mark `?`. For instance: `["?agent sees obj1",
"?agent rdf:type Human"]`.  Sets of triples that include variables are refered
to as *patterns* by `minimalkb`, and used as such in methods like `find`.

Instead of such *named* variables, you can also use `*` as an unnamed variable.
For instance, `kb["* rdf:type Agent"]` would return the list of all agents.

Note however that if you mix named and unnamed variables, only the *named*
variables will be returned: `kb["?agent looksAt *"]` would therefore return a
list of agents looking at 'something'.


### Models

Several methods take an optional `models` parameter. If set to `None` or to an
empty list, the method will update/query statements in the robot's base
cognitive model. If set to a list containing a single string `all`, all the
existing models are update/queried. Otherwise, you can pass a list of modelsyou
want to update/query.

### Methods

- `about(term, models=None)`: returns the list of triples where `term` is either
  subject, predicate or object
- `add(stmts, models=None, lifespan=0)`: adds statements to the given model(s)
  with the given lifespan. Alias for `revise` with `policy['method']='add'`
- `classesof(term, direct=False, models=None)`: returns the list of (direct
  of direct + indirect) classes of the given term
- `clear()`: reset the knowledge base, deleting all facts and all models
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
    would return: `[{"agent":"james", "action": "jumpHigh"}, {"agent": "laurel", "action":"jumpHigher"}]`
- `hello()`: returns the version number of the `minimalkb` server as a string. Can be used
  to check connection status.
- `load(filename, models=None)`: loads the content of the specified OWL ontology. Note that
  this capability is only available if
  [RDFlib](https://rdflib.readthedocs.io/en/stable/) is available to the server.
- `lookup(resource, models=None)`
- `methods()`: list available methods
- `remove(stmts, models=None)`: alias for `revise` with
  `policy['method']='retract'`.
- `revise(stmts, policy)`: Add/retract/updates one or several statements in the
  specified model.

  `policy` is a dictionary with the following fields:
  -  `method` (required): string in [`add`, `safe_add`, `retract`, `update`, `safe_update`, `revision`]
  -  `models` (optional, default to `None`): list of strings
  -  `lifespan` (optional, default to `0`): duration before automatic removal of statements, in
     seconds, float. If set to 0, statements do not expire.

- `sparql(query, model='default')`: performs a raw SPARQL query on a given
  model.  The SPARQL PREFIX and BASE are automatically added, no need to do it
  manually (even though you can if you want to use non-standard prefixes).

  Note that you are responsible for writing a syntactically corret SPARQL
  query. In particualar, all non-literal/non-variable terms must have a
  namespace (or a prefix).
  
  Results is returned as a JSON object that follow the standard [JSON
  serialization of SPARQL Results](https://www.w3.org/TR/2013/REC-sparql11-results-json-20130321/)

- `subscribe(type, trigger, var, patterns, models=None)`
- `update(stmts, models=None, lifespan=0)`: updates statements in the given model(s)
  with the given lifespan. Alias for `revise` with `policy['method']='update'`.
  If the predicate(s) are *not* inferred to be functional (i.e., it accept only
  one single value), behaves like `add`.

