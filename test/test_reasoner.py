#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import logging
import unittest
import time

try:
    import kb
except ImportError:
    import sys

    print("You must first install pykb")
    sys.exit(1)
from knowledge_core import __version__

from queue import Empty

REASONING_DELAY = 0.2


class TestRDFSReasoner(unittest.TestCase):
    def setUp(self):
        self.kb = kb.KB()
        self.kb.clear()

    def tearDown(self):
        self.kb.close()

    def test_basics(self):

        # OWL2 RL cls-thing
        self.assertTrue("owl:Thing rdf:type owl:Class" in self.kb)
        # OWL2 RL cls-nothing1
        self.assertTrue("owl:Nothing rdf:type owl:Class" in self.kb)

        # create new models
        self.kb.add(["s p o"], ["model1", "model2"])
        self.assertTrue(self.kb["owl:Thing rdf:type owl:Class", ["model1"]])
        self.assertTrue(self.kb["owl:Nothing rdf:type owl:Class", ["model1"]])
        self.assertTrue(self.kb["owl:Thing rdf:type owl:Class", ["model2"]])
        self.assertTrue(self.kb["owl:Nothing rdf:type owl:Class", ["model2"]])

    def test_complex_events_rdfs(self):

        evtid = self.kb.subscribe(["?a desires ?act", "?act rdf:type Action"], var="a")

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
        self.kb += ["Zorro rdfs:subClassOf Action"]
        time.sleep(REASONING_DELAY)
        # required to ensure the event is triggered after classification!
        self.kb += ["nop nop nop"]
        time.sleep(0.1)

        id, value = self.kb.events.get_nowait()
        self.assertEqual(id, evtid)
        self.assertCountEqual(value, [u"alfred"])

    def test_taxonomy_walking_inheritance(self):

        self.kb += ["john rdf:type Human"]
        self.assertCountEqual(self.kb.classesof("john"), [u"Human"])
        self.kb += ["Human rdfs:subClassOf Animal"]
        time.sleep(REASONING_DELAY)
        self.assertCountEqual(self.kb.classesof("john"), [u"Human", u"Animal"])
        self.assertCountEqual(self.kb.classesof("john", True), [u"Human"])
        self.kb -= ["john rdf:type Human"]
        time.sleep(REASONING_DELAY)
        self.assertFalse(self.kb.classesof("john"))

    def test_second_level_inheritance(self):

        self.kb += "myself rdf:type Robot"
        self.kb += [
            "Robot rdfs:subClassOf Agent",
            "Agent rdfs:subClassOf PhysicalEntity",
        ]

        time.sleep(REASONING_DELAY)

        self.assertTrue("Robot rdfs:subClassOf PhysicalEntity" in self.kb)
        self.assertTrue("myself rdf:type PhysicalEntity" in self.kb)

    def test_equivalent_classes_transitivity(self):

        self.kb += "myself rdf:type Robot"
        self.kb += [
            "Robot owl:equivalentClass Machine",
            "Machine owl:equivalentClass Automaton",
        ]
        self.kb += "PR2 rdfs:subClassOf Automaton"

        time.sleep(REASONING_DELAY)

        self.assertTrue("Robot owl:equivalentClass Automaton" in self.kb)
        self.assertCountEqual(
            self.kb.classesof("myself"), [u"Robot", u"Machine", u"Automaton"]
        )
        self.assertTrue("PR2 rdfs:subClassOf Robot" in self.kb)

    def test_existence_with_inference(self):

        self.kb += ["alfred rdf:type Human", "Human rdfs:subClassOf Animal"]
        time.sleep(REASONING_DELAY)
        self.assertTrue("alfred rdf:type Animal" in self.kb)

        self.kb += ["Animal rdfs:subClassOf Thing"]
        time.sleep(REASONING_DELAY)
        self.assertTrue("alfred rdf:type Thing" in self.kb)


def version():
    print("KnowledgeCore RDFS reasoner tests %s" % __version__)


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
