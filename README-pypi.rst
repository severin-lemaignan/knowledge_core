minimalKB
=========

minimalKB is a SQLite-backed minimalistic knowledge base, initially
designed for robots (in particular human-robot interaction or
multi-robot interaction).

It stores triples (like RDF/OWL triples), and provides an API accessible
via a simple socket protocol.

`pykb <https://github.com/severin-lemaignan/pykb>`__ provides an
idiomatic Python binding, making easy to integrate the knowledge base in
your applications.

It has almost no features, except it is fast and simple. Basic RDFS
reasoning is provided.

Written in Python. The only required dependency is ``sqlite3``. If
``rdflib`` is also available, you can easily import existing ontologies
in RDF/OWL/n3/Turtle formats in the knowledge base.

Documentation
-------------

You can use ``minimalkb`` either as a server, accessible from multiple
applications (clients), or in *embedded* mode (which does not require to
start a server process, but is limited to one single component). Note
that the embedded mode is only available for Python applciations.

In both case, and if your application is written in Python, it is highly
recommended to use `pykb <https://github.com/severin-lemaignan/pykb>`__
to interact the knowledge base.

Server mode
~~~~~~~~~~~

To start the knowledge base as a server, simply type:

::

   $ minimalkb

(run ``minimalkb --help`` for available options)

Then:

.. code:: python

   import kb

   with kb.KB() as kb:
       #...

See usage examples on the
`pykb <https://github.com/severin-lemaignan/pykb>`__ page, or in the
``minimalkb`` unit-tests.

Embedded mode
~~~~~~~~~~~~~

No need to start ``minimalkb``. Simply use the following code to start
using the knowledge base in your code:

.. code:: python

   import kb

   with kb.KB(embedded=True) as kb:
       #...

Interacting with the minimalkb from other languages
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

-  from C++: check
   `liboro <https://github.com/severin-lemaignan/liboro>`__
-  from any other language: the communication with the server relies on
   a simply socket-based text protocol. Feel free to get in touch if you
   need help to add support for your favourite language!

Features
--------

Server-Client or embedded
~~~~~~~~~~~~~~~~~~~~~~~~~

``minimalKB`` can be run as a stand-alone (socket) server, or directly
embedded in Python applications.

Multi-models
~~~~~~~~~~~~

``minimalKB`` is intended for dynamic environments, with possibly
several contexts/agents requiring separate knowledge models.

New models can be created at any time and each operation (like knowledge
addition/retractation/query) can operate on a specific subset of models.

Each models are also independently classified by the reasoner.

Event system
~~~~~~~~~~~~

``minimalKB`` provides a mechanism to *subscribe* to some conditions
(like: an instance of a given type is added to the knowledge base, some
statement becomes true, etc.) and get notified back.

Reasoning
~~~~~~~~~

``minimalKB`` only provides very basic RDFS/OWL reasoning capabilities:

-  it honors the transitive closure of the ``rdfs:subClassOf`` relation.
-  functional predicates (child of ``owl:functionalProperty``) are
   properly handled when updating the model (ie, if ``<S P O>`` is
   asserted with ``P`` a functional predicate, updating the model with
   ``<S P O'>`` will first cause ``<S P O>`` to be retracted).
-  ``owl:equivalentClass`` is properly handled.

The reasoner runs in its own thread, and classify the model at a given
rate, by default 5Hz. It is thus needed to wait ~200ms before the
results of the classification become visible in the model.

Transient knowledge
~~~~~~~~~~~~~~~~~~~

``minimalKB`` allows to attach ‘lifespans’ to statements: after a given
duration, they are automatically collected.

Ontology walking
~~~~~~~~~~~~~~~~

``minimalKB`` exposes several methods to explore the different
ontological models of the knowledge base. It is compatible with the
visualization tool
`oro-view <https://github.com/severin-lemaignan/oro-view>`__.
