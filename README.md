KnowledgeCore
==============

![KnowledgeCore icon](icon.png)

KnowledgeCore is a RDFlib-backed minimalistic knowledge base, initially designed
for robots (in particular human-robot interaction or multi-robot interaction).
It features full [ROS](https://www.ros.org) support.

It stores triples (like RDF/OWL triples), and provides an [API](doc/api.md)
accessible via a simple socket protocol or a [ROS wrapper](#ros-usage).

[pykb](https://github.com/severin-lemaignan/pykb) provides an idiomatic Python
binding over the socket interface, making easy to integrate the knowledge base in your application.
A similar API wrapper exists for ROS as well (see example below).

It integrates with the [reasonable](https://github.com/gtfierro/reasonable) OWL2
RL reasoner to provide OWL2 semantics and fast knowledge materialisation.

Example
-------

This example uses the ROS API (see below), with some Pythonic syntatic sugar:

```python

from knowledge_core.api import KB

rospy.init_node("test_knowledge_base")

kb = KB()

def on_robot_entering_antonio_property(evt):
  print("A robot entered Antonio's %s: %s" (evt[0]["place"], evt[0]["robot"]))

kb += "ari rdf:type Robot"  
kb += ["antonio looksAt ari", "ari isIn kitchen"]

kb.subscribe(["?robot isIn ?place", "?place belongsTo antonio", "?robot rdf:type Robot"], onRobotEnteringAntonioProperty)

kb += "kitchen belongsTo antonio"

# try as well:
# kb -= "antonio looksAt ari" to remove facts
# kb["* rdf:type Robot"] to query the knowledge base

rospy.spin()
```

will print:

```
A robot entered Antonio's kitchen: ari
```

Installation
------------

**KnowledgeCore only supports Python 3**

### Prerequisite

`rdflib >= 6.0.0`:

```
$ pip install rdflib
```

For reasoning (optional):

```
$ pip install reasonable
```

### Installation

From `pypi`:

```
$ pip install knowledge_core
```

From source:

```
$ git clone https://github.com/severin-lemaignan/knowledge_core.git
$ cd knowledge_core
$ python setup.py install
$ knowledge_core
```

If using ROS, you can also use your regular catkin workflow.


Documentation
-------------

### General usage

**If you are a roboticist, [jump to ROS usage](#ros-usage)**

You can use `KnowledgeCore` either as a server, accessible from multiple
applications (clients), or in *embedded* mode (which does not require to start a
server process, but is limited to one single client). Note that the embedded
mode is only available for Python applications.

In both case, and if your application is written in Python, it is highly recommended
to use [pykb](https://github.com/severin-lemaignan/pykb) to interact the
knowledge base.

#### Server mode


To start the knowledge base as a server, simply type:

```
$ knowledge_core
```

(run `knowledge_core --help` for available options)

Then:

```python
import kb

with kb.KB() as kb:
    #...
```

See usage examples on the [pykb](https://github.com/severin-lemaignan/pykb) page, or in the `KnowledgeCore` [unit-tests](testing).

#### Embedded mode

No need to start `KnowledgeCore`. Simply use the following code to start using the
knowledge base in your code:

```python
import kb

with kb.KB(embedded=True) as kb:
    #...
```

#### Interacting with KnowledgeCore from other languages

- from C++: check [liboro](https://github.com/severin-lemaignan/liboro)
- from any other language: the communication with the server relies on a simply
  socket-based text protocol. Feel free to get in touch if you need help to add
  support for your favourite language!

#### How do I get that fancy image on top of the README?

Check [oro-view](https://github.com/severin-lemaignan/oro-view) ;-)

### ROS usage

**Please first read the general [API introduction](doc/api.md), as this applies to the ROS interface as well.**

To start the ROS node:

```
rosrun knowledge_core knowledge_core
```

**Note that, in general, you want to use the 'Pythonic' wrapper built on top of the low-level ROS topics/services API. See example above. This Pythonic interface follows the [`pykb`](https://gitlab/interaction/pykb/) API (except in a few corner case that are not supported by the ROS interface).**

`knowledge_core` exposes two topics, `/kb/add_facts` and
`/kb/remove_facts`, to add/remove triples to the knowledge base. Both topics
expect a simple string with 3 tokens separated by spaces (if the object is a
literal string, use double quotes to escape it).

It also exposes the following services:

- `/kb/revise` to add/remove facts using a synchronous interface
- `/kb/query` to perform simple queries
- `/kb/sparql` to perform complex queries (full SPARQL end-point)
- `/kb/events` to subscribe to 'events' by providing a (set of) partially-bound
  triples. Calling the service returns an event *id*. Subscribe then to
  `/kb/events/<id>` to be notified everytime a new instance/class match the
  provided pattern
- `/kb/manage` to manage the knowledge base (including eg clearing all the
  facts)

Features
--------

### Server-Client or embedded

`KnowledgeCore` can be run as a stand-alone (socket) server, or directly embedded
in Python applications.

### Multi-models

`KnowledgeCore` is intended for dynamic environments, with possibly several
contexts/agents requiring separate knowledge models.

New models can be created at any time and each operation (like knowledge
addition/retractation/query) can operate on a specific subset of models.

Each models are also independently classified by the reasoner.

### Event system

`KnowledgeCore` provides a mechanism to *subscribe* to some conditions (like: an
instance of a given type is added to the knowledge base, some statement becomes
true, etc.) and get notified back.

### Reasoning

`KnowledgeCore` provides RDFS/OWL reasoning capabilities via the
[reasonable](https://github.com/gtfierro/reasonable) reasoner.

See [reasonable README](https://github.com/gtfierro/reasonable#owl-2-rules) for
the exact level of support of the different OWL2 RL rules.

### Transient knowledge

`KnowledgeCore` allows to attach 'lifespans' to statements: after a given duration,
they are automatically collected.

**[this functionality is currently disabled. Please open an issue of you need it
urgently]**

### Ontology walking

`KnowledgeCore` exposes several methods to explore the different ontological models
of the knowledge base. It is compatible with the visualization tool
[oro-view](https://github.com/severin-lemaignan/oro-view).

