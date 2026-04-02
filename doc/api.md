KnowledgeCore API
=================

KnowledgeCore exposes three interfaces, all backed by the same core engine:

1. **Internal Python API** (`knowledge_core.kb.KnowledgeCore`): direct access to
   the knowledge base. No server or ROS node needed.
2. **ROS 2 API** (`knowledge_core.api.KB`): the main supported interface for
   robotics applications. Wraps the ROS 2 topics and services with a Pythonic
   API.
3. **Socket API**: a TCP-based text protocol, compatible with
   [pykb](https://github.com/severin-lemaignan/pykb).

All three interfaces share the same concepts (statements, patterns, models,
events) described below.


Core concepts
-------------

### Statements

`KnowledgeCore` stores knowledge as RDF-like triples, using the infix syntax:
`subject predicate object`.  For instance: `sky0 hasColor blue`

Triples follow the [Turtle
syntax](https://www.w3.org/TR/turtle/#language-features):

- subjects and predicates must be valid RDF identifiers;
- objects can be either RDF identifiers or literals. String literals must be
  surrounded by double quotes (e.g. `"hello"`). Language tags (`"hello"@en`) and
  datatype annotations (`"10"^^xsd:integer`) are supported.
- RDF identifiers may include XML namespace prefixes, separated with a colon.
  For instance: `james rdf:type oro:Human`. The following prefixes are
  predefined:

  | Prefix | Namespace |
  |--------|-----------|
  | `oro`  | `http://kb.openrobots.org#` (default) |
  | `rdf`  | `http://www.w3.org/1999/02/22-rdf-syntax-ns#` |
  | `rdfs` | `http://www.w3.org/2000/01/rdf-schema#` |
  | `owl`  | `http://www.w3.org/2002/07/owl#` |
  | `xsd`  | `http://www.w3.org/2001/XMLSchema#` |

- the namespace prefix can be omitted altogether. In that case, the default
  OpenRobots Ontology prefix (`oro:`) is used. All new terms added without a
  prefix are added to this namespace.

### Patterns and variables

Triples may include *variables* (unbound terms). Variables must start with a
question mark `?`. For instance: `["?agent sees obj1", "?agent rdf:type
Human"]`.

Sets of triples that include variables are called *patterns*, and are used in
methods like `find`, `subscribe`, and the `[]` operator.

Instead of named variables, you can use `*` as an unnamed wildcard. For
instance, `kb["* rdf:type Agent"]` returns the list of all agents. Unnamed
wildcards are returned as `var1`, `var2`, etc.

If you mix named and unnamed variables, only the *named* variables are returned:
`kb["?agent looksAt *"]` returns a list of agents looking at 'something'.

### Models

Models are independent knowledge bases, meant for instance to store the
(estimated) knowledge of different agents interacting with the robot.

Several methods take an optional `models` parameter:
- if `None` or an empty list, the method operates on the default model;
- if a list of model names (strings), the method operates on those specific
  models. Non-existing models are created automatically.

Facts added to one model are not visible in other models.


Internal Python API
-------------------

The core `KnowledgeCore` class (`knowledge_core.kb.KnowledgeCore`) provides
direct access to the knowledge base without any network or ROS layer.

### Quick start

```python
from knowledge_core.kb import KnowledgeCore

kb = KnowledgeCore()                        # with OWL2 RL reasoner (if available)
kb = KnowledgeCore(enable_reasoner=False)   # without reasoner

kb += ["ari rdf:type Robot", "ari isIn kitchen"]
print(kb["* rdf:type Robot"])               # [{'var1': 'ari'}]
print("ari isIn kitchen" in kb)             # True
kb -= ["ari isIn kitchen"]
```

### Operators

The `KnowledgeCore` class supports the following Python operators:

| Operator | Example | Description |
|----------|---------|-------------|
| `+=`     | `kb += ["s p o"]` | Add/update statements |
| `-=`     | `kb -= ["s p o"]` | Remove statements |
| `[]`     | `kb["?x rdf:type Robot"]` | Query (returns list of dicts) |
| `in`     | `"ari rdf:type Robot" in kb` | Check existence (returns bool) |

The `[]` operator behaviour depends on the arguments:

- **Single 3-token pattern with variables**: returns a list of dictionaries
  with bindings. E.g. `kb["* rdf:type Robot"]` returns `[{'var1': 'ari'}]`.
- **Single 3-token pattern, fully bound**: returns the query results (truthy if
  the triple exists). E.g. `kb["ari rdf:type Robot"]` returns a non-empty list.
- **Multiple patterns**: returns matching variable bindings across all patterns.
  E.g. `kb["?a rdf:type Robot", "?a isIn ?place"]`.
- **A non-triple string**: performs a `lookup`. E.g. `kb["ari"]`.

An optional **models list** can be passed as the last argument:
`kb["?x rdf:type Robot", ["model1"]]`.

### Methods

#### Knowledge management

- **`update(stmts, models=None, lifespan=0)`**: add or update one or more
  statements.  If a statement's predicate is a functional property (i.e. it
  accepts only one value), the previous value is replaced. `lifespan` is the
  time in seconds before automatic removal (0 = no expiration).

- **`remove(stmts, models=None)`**: retract one or more statements. Supports
  wildcard patterns (e.g. `remove(["ari ?p ?o"])`) to retract all matching
  triples, but only with a single pattern at a time. Removing a non-existent
  statement is a silent no-op.

- **`revise(stmts, policy)`**: the general-purpose method for modifying the
  knowledge base. `policy` is a dictionary:
  - `method` (required): `'update'` or `'retract'`
  - `models` (optional): list of model names
  - `lifespan` (optional): duration in seconds (float), only for `update`

- **`clear(keep_defaults=False)`**: reset the knowledge base. If
  `keep_defaults=True`, re-loads the default ontologies after clearing.

- **`load(filename, models=None)`**: load an OWL/RDF ontology file into the
  given model(s). Supported formats: RDF/XML, N3, N-Triples, Turtle.

- **`save(path, basename='kb', models=None)`**: save the knowledge base to
  RDF/XML files. One file per model: `path/basename-<model>.rdf`.

#### Querying

- **`find(patterns, variables=None, models=None)`**: query the knowledge base.
  Returns a list of dictionaries with variable bindings. If `variables` is
  provided, only those variables are returned.

  ```python
  kb.find(["?person rdf:type Human", "?person likes ?food"])
  # [{'person': 'alice', 'food': 'pizza'}, {'person': 'bob', 'food': 'pasta'}]

  kb.find(["?person rdf:type Human", "?person likes ?food"], variables=["?food"])
  # [{'food': 'pizza'}, {'food': 'pasta'}]
  ```

- **`exist(stmts, models=None)`**: check whether all the given statements are
  asserted in the knowledge base. If the statements contain variables, runs a
  query and returns `True` if at least one result matches.

- **`about(term, models=None)`**: return all triples where `term` appears as
  subject, predicate, or object.

- **`lookup(term, models=None)`**: search for resources matching a string, both
  by name and by label. Returns a list of `(name, type)` tuples, where type is
  one of: `instance`, `class`, `object_property`, `datatype_property`,
  `literal`, `undecided`.

- **`label(term, models=None)`**: return the labels attached to a term, as a
  dictionary: `{"default": "...", "en": "...", "fr": "...", ...}`. The
  `default` key returns the English label, or the term name if no label exists.

- **`details(term, model=None)`**: return a rich dictionary with:
  - `id`: resource identifier
  - `label`: label dictionary (see `label()`)
  - `type`: one of `instance`, `class`, `object_property`, `datatype_property`, `undecided`
  - `attributes`: for classes: parents, children, instances; for instances: classes
  - `relations`: list of triples involving the term

- **`sparql(query, model=None)`**: execute a raw SPARQL query on a single model.
  PREFIX and BASE declarations are added automatically. Returns a JSON object
  following the [SPARQL Results JSON
  format](https://www.w3.org/TR/2013/REC-sparql11-results-json-20130321/).

  ```python
  res = kb.sparql("SELECT ?a WHERE { ?a :eats ?b . }")
  print(res['results']['bindings'])
  ```

#### Ontology introspection

- **`classesof(term, direct=False, models=None)`**: return the list of classes
  of the given term. If `direct=True`, only direct classes are returned (not
  inferred parent classes).

- **`hello()`**: return the name of the knowledge base server.

- **`version()`**: return the version string.

- **`methods()`**: return the list of available API methods.

#### Events

- **`subscribe(patterns, one_shot=False, models=None)`**: subscribe to an event.
  Every time the knowledge base is updated, `patterns` are evaluated. If they
  match, the event is fired. Returns an event ID (string).

  If `one_shot=True`, the event is automatically discarded after firing once.

  Subscribing to the same pattern twice returns the same event ID (event
  deduplication).

  *Note*: the event delivery mechanism depends on the interface (ROS topics,
  socket messages, or internal queues). See the interface-specific sections
  below.


ROS 2 API
---------

The ROS 2 interface is the main supported interface for robotics applications.

### Starting the node

```
ros2 launch knowledge_core knowledge_core.launch.py
```

Or with command-line arguments:

```
ros2 run knowledge_core knowledge_core --debug --no-reasoner
```

### Pythonic wrapper (`knowledge_core.api.KB`)

The recommended way to interact with the ROS 2 interface from Python is via the
`knowledge_core.api.KB` wrapper:

```python
from knowledge_core.api import KB

kb = KB()  # creates its own ROS node and executor
# or:
kb = KB(my_ros_node)  # reuses an existing node (must use MultiThreadedExecutor)

kb += ["ari rdf:type Robot"]
print(kb["* rdf:type Robot"])

def on_event(evt):
    print("Event:", evt)

kb.subscribe(["?robot rdf:type Robot"], on_event)
```

The `KB` class provides the same operators (`+=`, `-=`, `[]`, `in`) and methods
(`update`, `remove`, `find`, `about`, `lookup`, `label`, `details`, `sparql`,
`subscribe`, `clear`, `load`, `revise`) as the internal `KnowledgeCore` class,
but routes all calls through ROS 2 services.

Event callbacks are delivered via ROS 2 topic subscriptions.

### Low-level ROS 2 topics and services

**Topics** (in the `/kb` namespace):

| Topic | Type | Description |
|-------|------|-------------|
| `/kb/add_fact` | `std_msgs/String` | Add a single triple |
| `/kb/remove_fact` | `std_msgs/String` | Remove a single triple |
| `/kb/active_concepts` | `kb_msgs/ActiveConcepts` | Currently active concepts |
| `/kb/events/<id>` | `std_msgs/String` | Event notifications (one topic per event) |

**Services** (in the `/kb` namespace):

| Service | Type | Description |
|---------|------|-------------|
| `/kb/manage` | `kb_msgs/Manage` | Clear, load, save, status |
| `/kb/revise` | `kb_msgs/Revise` | Add/remove/update statements |
| `/kb/query` | `kb_msgs/Query` | Pattern-based queries |
| `/kb/about` | `kb_msgs/About` | All triples involving a term |
| `/kb/label` | `kb_msgs/About` | Label of a term |
| `/kb/details` | `kb_msgs/About` | Detailed info about a term |
| `/kb/lookup` | `kb_msgs/Lookup` | Full-text resource search |
| `/kb/sparql` | `kb_msgs/Sparql` | Raw SPARQL queries |
| `/kb/events` | `kb_msgs/KbEvent` | Subscribe to events |

The message types are defined in the
[kb_msgs](https://github.com/pal-robotics/kb_msgs/) package.


Socket API
----------

The socket API is a TCP-based text protocol, primarily used with the
[pykb](https://github.com/severin-lemaignan/pykb) Python client library.

### Starting the server

```
$ knowledge_core                    # default port 6969
$ knowledge_core --port 7000        # custom port
$ knowledge_core --no-ros           # without ROS support
$ knowledge_core --no-reasoner      # without OWL2 RL reasoner
$ knowledge_core --debug            # verbose logging
$ knowledge_core ontology.owl       # pre-load an ontology
```

When started, the server listens for TCP connections and also starts the ROS 2
node (unless `--no-ros` is passed). Both interfaces share the same underlying
`KnowledgeCore` instance.

### Protocol

Messages are terminated by `#end#`. Each request is a newline-separated message:

```
method_name
arg1_as_json
arg2_as_json
...
#end#
```

Responses are:

```
ok
result_as_json
#end#
```

or, on error:

```
error
kberror
error message
#end#
```

Events are pushed asynchronously:

```
event
event_id
event_content_as_json
#end#
```

### Using pykb

```python
import kb

with kb.KB() as kb:
    kb += ["ari rdf:type Robot"]
    print(kb["* rdf:type Robot"])
```

See the [pykb documentation](https://github.com/severin-lemaignan/pykb) for
details.
