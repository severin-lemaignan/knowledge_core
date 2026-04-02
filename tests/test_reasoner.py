#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2026 IIIA-CSIC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Unit tests for KnowledgeCore RDFS/OWL2 RL reasoning.

These tests require the 'reasonable' OWL2 RL reasoner. If not available,
all tests are skipped.

KNOWN LIMITATIONS of the 'reasonable' reasoner:
The following inferences are NOT currently supported by 'reasonable' and are
tested separately (marked with expectedFailure):

- rdfs11 (TBox subclass transitivity): if A rdfs:subClassOf B and
  B rdfs:subClassOf C, 'reasonable' does NOT infer A rdfs:subClassOf C.
  This is an RDFS entailment rule, not an OWL2 RL rule. The reasonable
  README notes RDFS entailment is TODO.
  However, instance-level inference still works: if x rdf:type A, then
  x rdf:type C IS correctly inferred (via cax-sco).

- equivalentClass transitivity: if A owl:equivalentClass B and
  B owl:equivalentClass C, 'reasonable' does NOT infer
  A owl:equivalentClass C. The eq-trans rule only handles owl:sameAs,
  not owl:equivalentClass. Instance-level inference works though
  (via cax-eqc1/cax-eqc2).

- equivalentClass -> subClassOf propagation: if A owl:equivalentClass B
  and C rdfs:subClassOf B, 'reasonable' does NOT infer
  C rdfs:subClassOf A. This would require scm-eqc rules (Schema
  Vocabulary) which are not implemented.
"""

from queue import Empty
import unittest

from knowledge_core import __version__
from knowledge_core.kb import KnowledgeCore

try:
    import reasonable  # noqa: F401
    HAS_REASONER = True
except ImportError:
    HAS_REASONER = False


@unittest.skipUnless(HAS_REASONER, 'reasonable reasoner not available')
class TestRDFSReasoner(unittest.TestCase):
    def setUp(self):
        self.kb = KnowledgeCore(enable_reasoner=True)

    def tearDown(self):
        pass

    ##########################################################
    # Tests for SUPPORTED reasoning capabilities
    ##########################################################

    def test_owl2_rl_axioms_on_new_models(self):
        """cls-thing/cls-nothing1 should also be available on new models."""
        self.kb.update(['s p o'], ['model1', 'model2'])
        self.assertTrue(
            self.kb.exist(
                ['owl:Thing rdf:type owl:Class'], models=['model1']
            )
        )
        self.assertTrue(
            self.kb.exist(
                ['owl:Nothing rdf:type owl:Class'], models=['model2']
            )
        )

    def test_instance_type_via_subclass(self):
        """rdfs9: if C1 rdfs:subClassOf C2 and x rdf:type C1, infer x rdf:type C2."""
        self.kb += ['alfred rdf:type Human', 'Human rdfs:subClassOf Animal']
        self.assertTrue('alfred rdf:type Animal' in self.kb)

    def test_instance_type_via_deep_subclass(self):
        """Instance type inference through a chain of subclass relations."""
        self.kb += ['alfred rdf:type Human', 'Human rdfs:subClassOf Animal']
        self.kb += ['Animal rdfs:subClassOf Thing']
        self.assertTrue('alfred rdf:type Thing' in self.kb)

    def test_second_level_inheritance(self):
        """Instance type inferred through 2-level subclass chain."""
        self.kb += ['myself rdf:type Robot']
        self.kb += [
            'Robot rdfs:subClassOf Agent',
            'Agent rdfs:subClassOf PhysicalEntity',
        ]

        self.assertTrue('myself rdf:type Agent' in self.kb)
        self.assertTrue('myself rdf:type PhysicalEntity' in self.kb)

    def test_equivalent_classes_instance_inference(self):
        """Instance type inferred through owl:equivalentClass."""
        self.kb += [
            'myself rdf:type Robot',
            'Robot owl:equivalentClass Machine',
            'Machine owl:equivalentClass Automaton',
        ]

        self.assertTrue('myself rdf:type Machine' in self.kb)
        self.assertTrue('myself rdf:type Automaton' in self.kb)

    def test_taxonomy_walking_with_reasoning(self):
        """Classesof returns inferred classes from subclass hierarchy."""
        self.kb += ['john rdf:type Human']
        self.assertIn('Human', self.kb.classesof('john'))

        self.kb += ['Human rdfs:subClassOf Animal']
        classes = self.kb.classesof('john')
        self.assertIn('Human', classes)
        self.assertIn('Animal', classes)

        # direct=True should only return the directly asserted class
        direct_classes = self.kb.classesof('john', direct=True)
        self.assertIn('Human', direct_classes)
        self.assertNotIn('Animal', direct_classes)

        self.kb -= ['john rdf:type Human']
        self.assertFalse(self.kb.classesof('john'))

    def test_subclass_triggers_inference_on_existing_instances(self):
        """Adding a subclass relation triggers type inference on existing instances."""
        self.kb += ['ragnagna rdf:type Zorro']
        self.assertFalse('ragnagna rdf:type Action' in self.kb)

        self.kb += ['Zorro rdfs:subClassOf Action']
        self.assertTrue('ragnagna rdf:type Action' in self.kb)

    def test_complex_event_with_reasoning(self):
        """Events should fire when reasoning infers new matching triples."""
        CLIENT = 'test_client'

        evt_id = self.kb.subscribe(
            ['?a desires ?act', '?act rdf:type Action']
        )
        self.kb.eventsubscriptions.setdefault(evt_id, []).append(CLIENT)

        # add a desire, but ragnagna is not yet an Action
        self.kb += ['alfred desires ragnagna']
        if CLIENT in self.kb.requestresults:
            with self.assertRaises(Empty):
                self.kb.requestresults[CLIENT].get_nowait()

        # ragnagna is a Zorro, still not an Action
        self.kb += ['ragnagna rdf:type Zorro']
        if CLIENT in self.kb.requestresults:
            with self.assertRaises(Empty):
                self.kb.requestresults[CLIENT].get_nowait()

        # Zorro rdfs:subClassOf Action -> reasoner infers ragnagna rdf:type Action
        # -> event should fire
        self.kb += ['Zorro rdfs:subClassOf Action']
        msg_type, evt = self.kb.requestresults[CLIENT].get_nowait()
        self.assertEqual(msg_type, 'event')

    def test_classesof_with_equivalent_classes(self):
        """Classesof should return equivalent classes as well."""
        self.kb += [
            'myself rdf:type Robot',
            'Robot owl:equivalentClass Machine',
            'Machine owl:equivalentClass Automaton',
        ]

        classes = self.kb.classesof('myself')
        self.assertIn('Robot', classes)
        self.assertIn('Machine', classes)
        self.assertIn('Automaton', classes)

    def test_owl2_rl_cls_thing(self):
        """OWL2 RL cls-thing: owl:Thing rdf:type owl:Class is materialised."""
        self.assertTrue('owl:Thing rdf:type owl:Class' in self.kb)

    def test_owl2_rl_cls_nothing(self):
        """OWL2 RL cls-nothing1: owl:Nothing rdf:type owl:Class is materialised."""
        self.assertTrue('owl:Nothing rdf:type owl:Class' in self.kb)

    ##########################################################
    # Tests for UNSUPPORTED reasoning capabilities
    #
    # These document known limitations of the 'reasonable'
    # reasoner. They are marked with expectedFailure so that:
    # - they don't break the test suite
    # - if 'reasonable' adds support later, the test will
    #   "unexpectedly pass" and we'll know to remove the mark
    ##########################################################

    @unittest.expectedFailure
    def test_tbox_subclass_transitivity(self):
        """
        Rdfs11: infer A rdfs:subClassOf C from A subClassOf B subClassOf C.

        NOT SUPPORTED by 'reasonable'. This is an RDFS entailment rule, not
        an OWL2 RL rule. The reasonable README notes that RDFS entailment
        semantics are TODO. Instance-level inference (x rdf:type A -> x
        rdf:type C) works correctly via cax-sco, but the TBox-level triple
        A rdfs:subClassOf C is not materialised.
        """
        self.kb += [
            'Robot rdfs:subClassOf Agent',
            'Agent rdfs:subClassOf PhysicalEntity',
        ]
        self.assertTrue(
            'Robot rdfs:subClassOf PhysicalEntity' in self.kb
        )

    @unittest.expectedFailure
    def test_equivalentclass_transitivity(self):
        """
        Infer A owl:equivalentClass C from A equivClass B equivClass C.

        NOT SUPPORTED by 'reasonable'. The eq-trans rule only handles
        owl:sameAs transitivity, not owl:equivalentClass. This would require
        scm-eqc1/scm-eqc2 (Schema Vocabulary rules) to decompose
        equivalentClass into mutual subClassOf, then rdfs11 for transitivity.
        """
        self.kb += [
            'Robot owl:equivalentClass Machine',
            'Machine owl:equivalentClass Automaton',
        ]
        self.assertTrue(
            'Robot owl:equivalentClass Automaton' in self.kb
        )

    @unittest.expectedFailure
    def test_equivalentclass_subclass_propagation(self):
        """
        Infer C rdfs:subClassOf A from A equivClass B and C subClassOf B.

        NOT SUPPORTED by 'reasonable'. Would require scm-eqc1/scm-eqc2
        (Schema Vocabulary rules, not implemented) to derive mutual
        subClassOf from equivalentClass.
        """
        self.kb += [
            'Robot owl:equivalentClass Machine',
            'Machine owl:equivalentClass Automaton',
            'PR2 rdfs:subClassOf Automaton',
        ]
        self.assertTrue('PR2 rdfs:subClassOf Robot' in self.kb)


def version():
    print('KnowledgeCore RDFS reasoner tests %s' % __version__)


if __name__ == '__main__':

    import argparse

    parser = argparse.ArgumentParser(
        description='Test suite for KnowledgeCore RDFS reasoner.'
    )
    parser.add_argument(
        '-v',
        '--version',
        action='version',
        version=version(),
        help='returns KnowledgeCore version',
    )
    parser.add_argument(
        '-f',
        '--failfast',
        action='store_true',
        help='stops at first failed test',
    )

    args = parser.parse_args()

    unittest.main(failfast=args.failfast)
