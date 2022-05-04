#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import logging
import unittest
import time

from kb import KbError

try:
    import kb
except ImportError:
    import sys

    print("You must first install pykb")
    sys.exit(1)

from knowledge_core import __version__

from queue import Empty

REASONING_DELAY = 0.2


class TestSequenceFunctions(unittest.TestCase):
    def setUp(self):
        self.kb = kb.KB()
        self.kb.clear()

    def tearDown(self):
        self.kb.close()

    def test_basics(self):

        self.kb.hello()

        with self.assertRaises(kb.KbError):
            self.kb.add()
        with self.assertRaises(kb.KbError):
            self.kb.add("toto")
        with self.assertRaises(kb.KbError):
            self.kb.add(["toto"])
        with self.assertRaises(kb.KbError):
            self.kb += ["toto"]
        with self.assertRaises(kb.KbError):
            self.kb += ["toto titi"]
        with self.assertRaises(kb.KbError):
            self.kb += ["toto titi tutu tata"]

    def test_basic_modifications(self):

        # check no exception is raised
        self.kb.add(["johnny rdf:type Human", 'johnny rdfs:label "A que Johnny"'])
        self.kb += ["alfred rdf:type Human", "alfred likes icecream"]
        self.kb.remove(["alfred rdf:type Human", "alfred likes icecream"])
        self.kb -= ["johnny rdf:type Human"]

        self.kb.revise(["toto likes tata"], {"method": "add"})

    def test_basic_model_modifications(self):

        # check no exception is raised
        self.kb.add(
            ["johnny rdf:type Human", 'johnny rdfs:label "A que Johnny"'], ["model1"]
        )
        self.kb.remove(["alfred rdf:type Human", "alfred likes icecream"], ["model1"])

        self.kb.add(
            ["johnny rdf:type Human", 'johnny rdfs:label "A que Johnny"'],
            ["model1", "model2"],
        )
        self.kb.remove(
            ["alfred rdf:type Human", "alfred likes icecream"], ["model1", "model2"]
        )

        self.kb.revise(["toto likes tata"], {"method": "add", "models": ["model1"]})
        self.kb.revise(
            ["toto likes tata"], {"method": "add", "models": ["model1", "model2"]}
        )

    def test_basic_kwargs(self):

        self.kb.revise(["toto likes tata"], policy={"method": "add"})
        self.kb.add(["johnny rdf:type Human"], lifespan=10)

    def test_modifications(self):

        self.assertCountEqual(
            self.kb["* * *"],
            [],
        )

        self.kb += ["alfred rdf:type Human"]
        self.assertCountEqual(
            self.kb["* * *"],
            [{"var1": "alfred", "var2": "rdf:type", "var3": "Human"}],
        )

        self.kb -= ["alfred rdf:type Human"]
        self.assertCountEqual(
            self.kb["* * *"],
            [],
        )

        self.kb += ["alfred rdf:type Human", "alfred likes icecream"]
        self.assertCountEqual(
            self.kb["* * *"],
            [
                {"var1": "alfred", "var2": "likes", "var3": "icecream"},
                {"var1": "alfred", "var2": "rdf:type", "var3": "Human"},
            ],
        )

    def test_existence(self):

        self.assertFalse("alfred" in self.kb)
        self.assertFalse("alfred likes icecream" in self.kb)
        self.assertFalse("alfred likes *" in self.kb)

        self.kb += ["alfred rdf:type Human", "alfred likes icecream"]

        self.assertTrue("alfred" in self.kb)
        self.assertFalse("tartempion" in self.kb)

        self.assertFalse("alfred likes" in self.kb)
        self.assertTrue("alfred likes icecream" in self.kb)
        self.assertTrue("alfred likes *" in self.kb)
        self.assertTrue("alfred likes ?smthg" in self.kb)
        self.assertTrue("* likes *" in self.kb)
        self.assertFalse("* dislikes *" in self.kb)
        self.assertTrue("?toto likes *" in self.kb)
        self.assertFalse("alfred likes mygrandmother" in self.kb)

        self.kb -= ["alfred rdf:type Human", "alfred likes icecream"]

        self.assertFalse("alfred likes icecream" in self.kb)
        self.assertFalse("alfred" in self.kb)

    def test_lookup(self):
        self.assertCountEqual(self.kb.lookup("alfred"), [])

        self.kb += ['alfred rdfs:label "alfred"']
        self.assertCountEqual(self.kb.lookup("alfred"), [["alfred", "undecided"]])
        self.assertCountEqual(
            self.kb.lookup("rdfs:label"), [["rdfs:label", "datatype_property"]]
        )

        self.kb += ["alfred rdf:type Robot"]
        self.assertCountEqual(self.kb.lookup("alfred"), [["alfred", "instance"]])
        self.assertCountEqual(self.kb.lookup("Robot"), [["Robot", "class"]])
        self.assertCountEqual(
            self.kb.lookup("rdf:type"), [["rdf:type", "object_property"]]
        )

        self.kb += ["alfred likes icecream"]
        self.assertCountEqual(self.kb.lookup("likes"), [["likes", "object_property"]])
        self.assertCountEqual(self.kb.lookup("alfred"), [["alfred", "instance"]])

        self.kb += ['nono rdfs:label "alfred"']
        self.assertCountEqual(
            self.kb.lookup("alfred"), [["alfred", "instance"], ["nono", "undecided"]]
        )

        self.kb += ['gerard rdfs:label "likes"']
        self.assertCountEqual(
            self.kb.lookup('"likes"'),
            [['"likes"', "literal"]],
        )

        self.kb += ["gerard age 18"]
        self.assertCountEqual(self.kb.lookup("age"), [["age", "datatype_property"]])

    def test_literals(self):

        literals = [
            '"test"',
            '"""test"""',
            '"""toto\ntata"""',
            '"test"@fr',
            '"foo"^^<http://example.org/my/datatype>',
            '"""10"""^^xsd:decimal',
            "-5",
            "0",
            "1",
            "10",
            "+1",
            '"-5"^^xsd:integer',
            '"10"^^<http://www.w3.org/2001/XMLSchema#integer>',
            "1.3e2",
            "10e0",
            "-12.5e10",
            '"1.3e2"^^xsd:double',
            '"-12.5e10"^^<http://www.w3.org/2001/XMLSchema#double>',
            "0.0",
            "1.0",
            "1.234567890123456789",
            "-5.0",
            '"0.0"^^xsd:decimal',
            '"-5.0"^^<http://www.w3.org/2001/XMLSchema#decimal>',
            "true",
            "false",
            '"true"^^xsd:boolean',
            '"false"^^<http://www.w3.org/2001/XMLSchema#boolean>',
        ]

        malformed = ["'test'", '"toto\ntata"']

        objects = ["test", "False", "True"]

        for i, val in enumerate(literals):
            self.kb += ["robert rel%s %s" % (i, val)]
            self.assertCountEqual(
                self.kb.lookup("rel%s" % i), [["rel%s" % i, "datatype_property"]]
            )

        for i, val in enumerate(objects):
            self.kb += ["robert relobj%s %s" % (i, val)]
            self.assertCountEqual(
                self.kb.lookup("relobj%s" % i), [["relobj%s" % i, "object_property"]]
            )

        for val in malformed:
            with self.assertRaises(kb.KbError):
                self.kb += ["robert rel%s %s" % (i, val)]

    def test_retrieval(self):

        self.assertFalse(self.kb.about("Human"))
        self.assertFalse(self.kb["* rdf:type Human"])

        self.kb += ["johnny rdf:type Human", 'johnny rdfs:label "A que Johnny"']
        self.kb += ["alfred rdf:type Human", "alfred likes icecream"]

        self.assertCountEqual(
            self.kb.about("Human"),
            [["johnny", "rdf:type", "Human"], ["alfred", "rdf:type", "Human"]],
        )

        self.assertCountEqual(
            self.kb["* rdf:type Human"], [{"var1": "johnny"}, {"var1": "alfred"}]
        )

        self.kb -= ["alfred rdf:type Human", "alfred likes icecream"]

        self.assertCountEqual(self.kb["* rdf:type Human"], [{"var1": "johnny"}])

        self.assertTrue(self.kb["johnny rdf:type Human"])

    def test_models_retrieval(self):

        self.kb.add(["s p o", "s2 p2 o2"], ["model1", "model2"])

        self.assertTrue(self.kb["s p o", ["model1"]])
        self.assertTrue(self.kb["s2 p2 o2", ["model1"]])
        self.assertTrue(self.kb["s p o", ["model2"]])

    def test_complex_queries(self):
        self.assertCountEqual(
            self.kb["?agent rdf:type Robot", "?agent desires ?obj"], []
        )

        self.kb += ["nono rdf:type Human", "alfred rdf:type Robot"]
        self.assertCountEqual(
            self.kb["* * *"],
            [
                {"var1": "nono", "var2": "rdf:type", "var3": "Human"},
                {"var1": "alfred", "var2": "rdf:type", "var3": "Robot"},
            ],
        )

        self.kb += ["nono desires jump", "alfred desires oil"]
        self.assertCountEqual(
            self.kb["* * *"],
            [
                {"var1": "nono", "var2": "desires", "var3": "jump"},
                {"var1": "alfred", "var2": "desires", "var3": "oil"},
                {"var1": "nono", "var2": "rdf:type", "var3": "Human"},
                {"var1": "alfred", "var2": "rdf:type", "var3": "Robot"},
            ],
        )

        self.kb += ["nono loves icecream"]
        self.assertEqual(
            self.kb["?agent desires jump", "?agent loves icecream"], [{"agent": "nono"}]
        )
        self.assertCountEqual(
            self.kb["?agent desires *", "?agent loves icecream"], [{"agent": "nono"}]
        )

        self.kb += ["jump rdf:type Action"]
        self.assertCountEqual(
            self.kb["?agent rdf:type Robot", "?agent desires ?obj"],
            [{"agent": "alfred", "obj": "oil"}],
        )
        self.assertCountEqual(
            self.kb["?agent desires ?act", "?act rdf:type Action"],
            [{"agent": "nono", "act": "jump"}],
        )
        self.assertCountEqual(
            self.kb["?agent desires ?obj"],
            [{"agent": "alfred", "obj": "oil"}, {"agent": "nono", "obj": "jump"}],
        )

    def test_update(self):
        self.kb += ["nono isNice true", "isNice rdf:type owl:FunctionalProperty"]
        self.assertCountEqual(self.kb["* isNice true"], [{"var1": "nono"}])

        self.kb += ["nono isNice false"]
        self.assertFalse(self.kb["* isNice true"])
        self.assertCountEqual(self.kb["* isNice false"], [{"var1": "nono"}])

    def test_about(self):
        self.kb.add(["nono isNice true"], ["model1"])
        self.kb.add(["jamesbond isNice true"], ["model2"])
        self.assertTrue(bool(self.kb.about("nono", ["model1"])))
        self.assertFalse(bool(self.kb.about("nono", ["model2"])))

    def test_events(self):

        eventtriggered = [False]

        def onevent(evt):
            # py3 only! -> workaround is turning eventtriggered into a list
            # nonlocal eventtriggered
            print("In callback. Got evt %s" % evt)
            eventtriggered[0] = True

        evtid = self.kb.subscribe(["?o isIn room"], onevent)

        # should not trigger an event
        self.kb += ["alfred isIn garage"]
        time.sleep(0.1)
        self.assertFalse(eventtriggered[0])

        # should trigger an event
        self.kb += ["alfred isIn room"]
        time.sleep(0.1)
        self.assertTrue(eventtriggered[0])

        eventtriggered[0] = False

        # should not trigger an event
        self.kb += ["alfred leaves room"]
        time.sleep(0.1)
        self.assertFalse(eventtriggered[0])

        # alfred is already in garage, should not fire an event
        evtid2 = self.kb.subscribe(["?o isIn garage"], onevent)
        self.assertFalse(evtid == evtid2)
        time.sleep(0.1)
        self.assertFalse(eventtriggered[0])

        # alfred is already in garage, should not fire an event
        self.kb += ["alfred isIn garage"]
        time.sleep(0.1)
        self.assertFalse(eventtriggered[0])

        self.kb += ["batman isIn garage"]
        time.sleep(0.1)
        self.assertTrue(eventtriggered[0])

    def test_events_multiclients(self):

        with kb.KB() as kb2:

            eventtriggered = [False]

            def onevent(evt):
                # py3 only! -> workaround is turning eventtriggered into a list
                # nonlocal eventtriggered
                print("In callback. Got evt %s" % evt)
                eventtriggered[0] = True

            self.kb.subscribe(["?o isIn room"], onevent)

            # should not trigger an event
            kb2 += ["alfred isIn garage"]
            time.sleep(0.1)
            self.assertFalse(eventtriggered[0])

            # should trigger an event
            kb2 += ["alfred isIn room"]
            time.sleep(0.1)
            self.assertTrue(eventtriggered[0])

    def test_polled_events(self):

        evtid = self.kb.subscribe(["?o isIn room"])

        # should not trigger an event
        self.kb += ["alfred isIn garage"]
        time.sleep(0.1)

        with self.assertRaises(Empty):
            self.kb.events.get_nowait()

        # should trigger an event
        self.kb += ["alfred isIn room"]
        time.sleep(0.1)

        id, value = self.kb.events.get_nowait()
        self.assertEqual(id, evtid)
        self.assertCountEqual(value, [{"o": "alfred"}])

        # should not trigger an event
        self.kb += ["alfred leaves room"]
        time.sleep(0.1)

        with self.assertRaises(Empty):
            self.kb.events.get_nowait()

        # alfred is already in garage, should not fire an event
        evtid = self.kb.subscribe(["?o isIn garage"])
        time.sleep(0.1)

        with self.assertRaises(Empty):
            self.kb.events.get_nowait()

        # alfred is already in garage, should not fire an event
        self.kb += ["alfred isIn garage"]
        time.sleep(0.1)

        with self.assertRaises(Empty):
            self.kb.events.get_nowait()

        self.kb += ["batman isIn garage"]
        time.sleep(0.1)

        id, value = self.kb.events.get_nowait()
        self.assertEqual(id, evtid)
        self.assertCountEqual(value, [{"o": "batman"}])

    def test_complex_events(self):

        evtid = self.kb.subscribe(["?a desires ?act", "?act rdf:type Action"])

        # should not trigger an event
        self.kb += ["alfred desires ragnagna"]
        time.sleep(0.1)

        with self.assertRaises(Empty):
            self.kb.events.get_nowait()

        # should not trigger an event
        self.kb += ["ragnagna rdf:type Zorro"]
        time.sleep(0.1)

        with self.assertRaises(Empty):
            self.kb.events.get_nowait()

        # should trigger an event
        self.kb += ["ragnagna rdf:type Action"]
        time.sleep(0.1)

        id, value = self.kb.events.get_nowait()
        self.assertEqual(id, evtid)
        self.assertEqual(value, [{"a": "alfred", "act": "ragnagna"}])

    def test_taxonomy_walking(self):

        self.assertFalse(self.kb.classesof("john"))
        self.kb += ["john rdf:type Human"]
        self.assertCountEqual(self.kb.classesof("john"), [u"Human"])
        self.kb += ["john rdf:type Genius"]
        self.assertCountEqual(self.kb.classesof("john"), [u"Human", u"Genius"])
        self.kb -= ["john rdf:type Human"]
        self.assertCountEqual(self.kb.classesof("john"), [u"Genius"])

    def test_memory(self):

        self.kb.add(["john rdf:type Human"])
        time.sleep(1.2)
        self.assertTrue("john" in self.kb)
        self.kb.remove(["john rdf:type Human"])
        self.kb.add(["john rdf:type Human"], [], 0.5)
        time.sleep(0.4)
        self.assertTrue("john" in self.kb)
        time.sleep(0.6)
        self.assertFalse("john" in self.kb)


def version():
    print("KnowledgeCore tests %s" % __version__)


if __name__ == "__main__":

    import argparse

    parser = argparse.ArgumentParser(description="Test suite for KnowledgeCore.")
    parser.add_argument(
        "-v",
        "--version",
        action="version",
        version=version(),
        help="returns KnowledgeCore version",
    )
    parser.add_argument(
        "-f", "--failfast", action="store_true", help="stops at first failed test"
    )

    args = parser.parse_args()

    kblogger = logging.getLogger("kb")
    console = logging.StreamHandler()
    kblogger.setLevel(logging.DEBUG)
    kblogger.addHandler(console)

    unittest.main(failfast=args.failfast)
    # unittest.main(failfast=True)
