^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package knowledge_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.8.0 (2022-08-17)
------------------
* fix KB.find to properly handle ROS queries with no variable specified
  It now returns all present variables instead of an error
* output literal as python/JSON literal instead of XSD ones
* minor debug msg changes
* Contributors: Séverin Lemaignan

2.7.0 (2022-07-20)
------------------
* add missing space in SPAQRL prefixes
* [api] expose 'stats' to the ROS py wrapper
* [minor] improve debug message experience
* Update README.md with example of how to use the ROS API
* increase logging level of several msgs
* fix corner case where trying to publish to unregistered evt topic
* Contributors: Séverin Lemaignan

2.6.1 (2022-06-13)
------------------
* [api] add (back) ability to save the knowledge base
* Contributors: Séverin Lemaignan

2.6.0 (2022-06-13)
------------------

Major changes:

* [api] pythonic API over ROS
  To use:
  ```
  from knowledge_core.api import KB
  kb = KB()
  ```
  then:
  ```
  kb += "ari rdf:type Human"
  kb -= "ari rdf:type Human"
  if "ari rdf:type Human":
  print("come one, ARI is not a human!")
  def on_robot(evt):
  print("new robots! %s" % evt)
  kb.subscribe("* rdf:type Robot", on_robot)
  kb += ["tiago rdf:type Robot", "ari rdf:type Robot"]
  ```
  ...and more
* support events via ROS service + topics
  One subscribe to an event via the /kb/events [Event.srv] service.
  The service returns an event id corresponding to a topic (eg
  /kb/events/evt_1234) that the client can subscribe to to be notified
  when the event triggers.
  /examples/kb_events_ros.py provides an example.
* [api] add support for the 'lookup' API
* [api] add support for the 'about' API

Other changes:

* [api] fix inconsistency in return type of 'find'
* [doc] briefly document ROS support
* set the asyncore loop timeout to 20ms for much more responsive events

* Contributors: Séverin Lemaignan

2.5.4 (2022-05-30)
------------------
* add missing dep on message_runtime
* move ROS back to own file + add support for adding/removing facts via topics
* [api] add now an alias for update
* Contributors: Séverin Lemaignan


2.5.3 (2022-05-30)
------------------
* remove ontologies from this repo
  Ontologies are stored in an independent repo (oro)
* minor maintainance (doc, copyright...)
* Contributors: Séverin Lemaignan

2.5.2 (2022-05-04)
------------------
* add back missing files following project renaming
* Contributors: Séverin Lemaignan

2.5.1 (2022-05-04)
------------------
* provide better feedback when running outside of ROS
* more minimalkb->KnowledgeCore
* Contributors: Séverin Lemaignan

2.5.0 (2022-05-04)
------------------
* rename project to KnowledgeCore
* setup.py: install bin so that the project remains usable via pip
* Contributors: Séverin Lemaignan

2.4.0 (2022-04-29)
------------------
* increase coverage of ROS Query test + only test reasoning if reasoner available
* kb.version + export whether reasoner available through ROS Manage service
* Contributors: Séverin Lemaignan

2.3.0
-----

*Released on 12 Apr 2022*

-  Add ROS support via Revise, Query, Sparql, Manage services

2.2.1
-----

*Released on 30 Mar 2022*

-  better error handling/reporting
-  slightly stricter statement parsing
-  updated LICENSE to 2022
-  remove requirement for rdflib >= 6.0.0 (works with RDFlib 4.x.x)

2.2.0
-----

*Released on 30 Mar 2022*

-  change find() API to make it more consistent

   -  whatever the number of variables, always returns a list of dict
      with the variables as key
   -  the param ‘vars’ is now optional.
   -  if no variable is passed, search for all variables by default
   -  anonymous variables (starting by ``__``) are now named var1,
      var2,… in the results

-  removed unused ``minimalkb/services`` sub-package

2.1.0
-----

*Released on 29 Mar 2022*

-  add ``kb.label`` to retrieve (multi-lingual) labels associated to a
   term
-  add support for querying direct classes, instances, subclasses,
   superclasses (via eg ``kb.details``)
-  significantly simplify the event subscription mechanism. Now, you
   simply pass a list of patterns with an optional ``one_shot``
   parameter (false by default)
-  better `API
   documentation <https://github.com/severin-lemaignan/minimalkb/blob/master/doc/api.md>`__
-  remove ``kb.check`` as it was strictly equivalent to ``kb.exist``
-  fix bug where loaded ontologies would not be processed by the
   reasoner
-  fix internal methods being mistakenly exposed in the API
-  a few other minor bug fixes

2.0.0
-----

**Attention: minimalkb-2.0 is a major rewrite of minimalkb, and only
partially backward-compatible with minimalkb-1.x.x**

-  complete rewrite of the core, around rdflib-6.x.x
-  as a consequence, much more standard-compilant parsing of statements
-  integration with
   ```reasonable`` <https://github.com/gtfierro/reasonable>`__ for fast
   OWL2 RL reasoning.
-  a new ``sparql`` API method has been add, to perform raw SPARQL
   queries.

Porting notes from 1.x.x:

-  old compatibility (``compat``) API methods have been removed
-  the ``find`` API method does not take a ``constraints`` parameter any
   more
-  ``retract`` has been renamed to ``remove``

1.2.1
-----

-  fix a corner case where conversion of literals to Python object was
   actually incorrect

1.2.0
-----

-  literal are now converted to their respective Python type (not yet
   done for XSD literal like “10^^xsd:integer”)

1.1.2
-----

Main changes since 1.1.2:

-  create custom readme to please pypi

1.1.1
-----

Main changes since 1.1.0:

-  fix issue preventing the loading of external ontologies

1.1.0
-----

Main changes since 1.0.0:

-  fix issue with SQLite triple store where statements were uniquely
   identified by an unstable hash algorithm.

1.0.0
-----

Main changes since 0.9:

-  port to python3
-  added (some) documentation
-  api: load() can take option ‘models’ parameter
-  provide more exhaustive API documentation with starting with –debug
-  added support for OWL2 RL rules cls-thing and cls-nothing1

0.9.0
-----

Main changes since 0.8.1:

-  fixes + doc in the reasoning engine
-  clean-up of unit-tests (cf current results below)
-  a few other minor bugfixes


Unit-tests results:

-  ``testing/test.py``: 17 tests, 1 expected error (unsupported feature:
   multiple var in multi-pattern queries) , 0 failure
-  ``testing/test_reasoner.py``: 5 tests, 0 errors, 0 failures
-  ``testing/test_embedded.py``: 17 tests, 4 errors, 3 failures


0.8.1
-----

Main changes since 0.8:

-  version correctly bumped ;-)

0.8.0
-----

Main changes since 0.7:

-  improved a bit behaviour of memoization: ‘undecided’ reasoning
   results are not memoized anymore; ‘clear’ also clear memoized results
-  use only the default model (previously all) when no model is
   specified
-  ‘lookup’ request has been significantly improved, in particular by
   taking into account labels.
-  ‘check’ has a first partial implementation (by just checking if
   checked statement are already asserted)
-  fix a bad bug leading to models being ignored in an ‘about’ request
-  fix a race with the reasoner when clearing the database



0.7.0
-----

Main changes since 0.6:

-  fix a serious issue with events (that were not triggered more than
   once)
-  support the NEW_CLASS_EVENT properly
-  slightly improved the reasoning capabilities: every models are now
   correctly classified, owl:equivalentClass and a few common owl
   symmetric predicates (owl:sameAs, owl:differentFrom) are handled to
   some extend.

Also, compatibility with the C++ liboro bindings (hence, oro-view) has
been extended to events.


0.6.0
-----

Main changes:

-  many improvements to the way minimalKB deals with ending/closing
-  support for ‘embedding’ minimalKB in a Python application (via
   ‘pykb.KB’ constructor option)
-  support for RPC call with keyword arguments
-  a couple of other bugs smashed out.

0.5.0
-----

Main changes: - support for functional properties (new statements
replace existing values instead of adding new facts) - fixed a bug in
event id generation that broke the event system - a handful of other
minor fixes

0.4.0
-----

Main new features:

-  fixed incorrect event dispatching when more than one client
-  support statements with limited lifespan
-  much extended taxonomy walking (oro-view now works with minimalKB)

0.3.0
-----

Main changes:

-  large rewrite of the query management (still far from complete,
   though).
-  as a consequence, better handling of complex queries like: [?a
   desires ?b, ?b type Action]
-  existence check can now handle any pattern
-  support of OWL/RDF/turtle/n3 loading when RDFlib is available
-  added basic command-line options

0.2.0
-----



Main changes: - addition of a simple RDFS reasoner (supports only
rdf:type and rdfs:subClassOf for now) - initial work for a RDFlib
backend

0.1.0
-----

First release of minimalKB

-  only a simple SQLite backend
-  run (but do not pass!) all ~300 Dialogs unit-tests, both with pyoro
   and pykb.
