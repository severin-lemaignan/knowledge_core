minimalKB
=========

![Screenshot of a minimalKB knowledge model viewed with oro-view](doc/oroview.jpg)

minimalKB is a RDFlib-backed minimalistic knowledge base, initially designed
for robots (in particular human-robot interaction or multi-robot interaction).

It stores triples (like RDF/OWL triples), and provides an [API](doc/api.md)
accessible via a simple socket protocol.

[pykb](https://github.com/severin-lemaignan/pykb) provides an idiomatic Python
binding, making easy to integrate the knowledge base in your applications.

It integrates with the [reasonable](https://github.com/gtfierro/reasonable) OWL2
RL reasoner to provide OWL2 semantics and fast knowledge materialisation.


Installation
------------

**minimalkb only supports Python 3**

### Prerequisite

`rdlib >= 6.0.0`:

```
$ pip install rdflib
```

For reasoning:

```
$ pip install reasonable
```

### Installation

From `pypi`:

```
$ pip install minimalkb
```

From source:

```
$ git clone https://github.com/severin-lemaignan/minimalkb.git
$ cd minimalkb
$ python setup.py install
$ minimalkb
```


Documentation
-------------

You can use `minimalkb` either as a server, accessible from multiple
applications (clients), or in *embedded* mode (which does not require to start a
server process, but is limited to one single component). Note that the embedded
mode is only available for Python applciations.

In both case, and if your application is written in Python, it is highly recommended
to use [pykb](https://github.com/severin-lemaignan/pykb) to interact the
knowledge base.

### Server mode


To start the knowledge base as a server, simply type:

```
$ minimalkb
```

(run `minimalkb --help` for available options)

Then:

```python
import kb

with kb.KB() as kb:
    #...
```

See usage examples on the [pykb](https://github.com/severin-lemaignan/pykb) page, or in the `minimalkb` [unit-tests](testing).

### Embedded mode

No need to start `minimalkb`. Simply use the following code to start using the
knowledge base in your code:

```python
import kb

with kb.KB(embedded=True) as kb:
    #...
```

### Interacting with the minimalkb from other languages

- from C++: check [liboro](https://github.com/severin-lemaignan/liboro)
- from any other language: the communication with the server relies on a simply
  socket-based text protocol. Feel free to get in touch if you need help to add
  support for your favourite language!

### How do I get that fancy image on top of the README?

Check [oro-view](https://github.com/severin-lemaignan/oro-view) ;-)

Features
--------

### Server-Client or embedded

`minimalKB` can be run as a stand-alone (socket) server, or directly embedded
in Python applications.

### Multi-models

`minimalKB` is intended for dynamic environments, with possibly several
contexts/agents requiring separate knowledge models.

New models can be created at any time and each operation (like knowledge
addition/retractation/query) can operate on a specific subset of models.

Each models are also independently classified by the reasoner.

### Event system

`minimalKB` provides a mechanism to *subscribe* to some conditions (like: an
instance of a given type is added to the knowledge base, some statement becomes
true, etc.) and get notified back.

### Reasoning

`minimalKB` provides RDFS/OWL reasoning capabilities via the
[reasonable](https://github.com/gtfierro/reasonable) reasoner.

See [reasonable README](https://github.com/gtfierro/reasonable#owl-2-rules) for
the exact level of support of the different OWL2 RL rules.

### Transient knowledge

`minimalKB` allows to attach 'lifespans' to statements: after a given duration,
they are automatically collected.

**[this functionality is currently unavailable]**

### Ontology walking

`minimalKB` exposes several methods to explore the different ontological models
of the knowledge base. It is compatible with the visualization tool
[oro-view](https://github.com/severin-lemaignan/oro-view).

