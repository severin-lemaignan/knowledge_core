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

"""Unit tests for KnowledgeCore (non-ROS, direct API)."""

from queue import Empty
import time
import unittest

from knowledge_core import __version__
from knowledge_core.exceptions import KbServerError
from knowledge_core.kb import KnowledgeCore

REASONING_DELAY = 0.2


class TestKnowledgeCore(unittest.TestCase):
    def setUp(self):
        self.kb = KnowledgeCore(enable_reasoner=False)

    def tearDown(self):
        pass

    def test_basics(self):

        self.kb.hello()

        with self.assertRaises(KbServerError):
            self.kb.update(['toto'])
        with self.assertRaises(KbServerError):
            self.kb.update(['toto titi'])
        with self.assertRaises(KbServerError):
            self.kb.update(['toto titi tutu tata'])

    def test_basic_modifications(self):

        # check no exception is raised
        self.kb.update(
            ['johnny rdf:type Human', 'johnny rdfs:label "A que Johnny"']
        )
        self.kb += ['alfred rdf:type Human', 'alfred likes icecream']
        self.kb.remove(['alfred rdf:type Human', 'alfred likes icecream'])
        self.kb -= ['johnny rdf:type Human']

        self.kb.revise(['toto likes tata'], {'method': 'update'})

    def test_basic_model_modifications(self):

        # check no exception is raised
        self.kb.update(
            ['johnny rdf:type Human', 'johnny rdfs:label "A que Johnny"'],
            ['model1'],
        )
        self.kb.remove(
            ['alfred rdf:type Human', 'alfred likes icecream'], ['model1']
        )

        self.kb.update(
            ['johnny rdf:type Human', 'johnny rdfs:label "A que Johnny"'],
            ['model1', 'model2'],
        )
        self.kb.remove(
            ['alfred rdf:type Human', 'alfred likes icecream'],
            ['model1', 'model2'],
        )

        self.kb.revise(
            ['toto likes tata'], {'method': 'update', 'models': ['model1']}
        )
        self.kb.revise(
            ['toto likes tata'],
            {'method': 'update', 'models': ['model1', 'model2']},
        )

    def test_basic_kwargs(self):

        self.kb.revise(['toto likes tata'], policy={'method': 'update'})
        self.kb.update(['johnny rdf:type Human'], lifespan=10)

    def test_modifications(self):

        self.assertCountEqual(
            self.kb['* * *'],
            [],
        )

        self.kb += ['alfred rdf:type Human']
        self.assertCountEqual(
            self.kb['* * *'],
            [{'var1': 'alfred', 'var2': 'rdf:type', 'var3': 'Human'}],
        )

        self.kb -= ['alfred rdf:type Human']
        self.assertCountEqual(
            self.kb['* * *'],
            [],
        )

        self.kb += ['alfred rdf:type Human', 'alfred likes icecream']
        self.assertCountEqual(
            self.kb['* * *'],
            [
                {'var1': 'alfred', 'var2': 'likes', 'var3': 'icecream'},
                {'var1': 'alfred', 'var2': 'rdf:type', 'var3': 'Human'},
            ],
        )

    def test_existence(self):

        self.assertFalse('alfred' in self.kb)
        self.assertFalse('alfred likes icecream' in self.kb)
        self.assertFalse('alfred likes *' in self.kb)

        self.kb += ['alfred rdf:type Human', 'alfred likes icecream']

        self.assertTrue('alfred' in self.kb)
        self.assertFalse('tartempion' in self.kb)

        self.assertFalse('alfred likes' in self.kb)
        self.assertTrue('alfred likes icecream' in self.kb)
        self.assertTrue('alfred likes *' in self.kb)
        self.assertTrue('alfred likes ?smthg' in self.kb)
        self.assertTrue('* likes *' in self.kb)
        self.assertFalse('* dislikes *' in self.kb)
        self.assertTrue('?toto likes *' in self.kb)
        self.assertFalse('alfred likes mygrandmother' in self.kb)

        self.kb -= ['alfred rdf:type Human', 'alfred likes icecream']

        self.assertFalse('alfred likes icecream' in self.kb)
        self.assertFalse('alfred' in self.kb)

    def test_lookup(self):
        self.assertCountEqual(self.kb.lookup('alfred'), [])

        self.kb += ['alfred rdfs:label "alfred"']
        self.assertCountEqual(
            self.kb.lookup('alfred'), [('alfred', 'undecided')]
        )
        self.assertCountEqual(
            self.kb.lookup('rdfs:label'),
            [('rdfs:label', 'datatype_property')],
        )

        self.kb += ['alfred rdf:type Robot']
        self.assertCountEqual(
            self.kb.lookup('alfred'), [('alfred', 'instance')]
        )
        self.assertCountEqual(
            self.kb.lookup('Robot'), [('Robot', 'class')]
        )
        self.assertCountEqual(
            self.kb.lookup('rdf:type'), [('rdf:type', 'object_property')]
        )

        self.kb += ['alfred likes icecream']
        self.assertCountEqual(
            self.kb.lookup('likes'), [('likes', 'object_property')]
        )
        self.assertCountEqual(
            self.kb.lookup('alfred'), [('alfred', 'instance')]
        )

        self.kb += ['nono rdfs:label "alfred"']
        self.assertCountEqual(
            self.kb.lookup('alfred'),
            [('alfred', 'instance'), ('nono', 'undecided')],
        )

        self.kb += ['gerard rdfs:label "likes"']
        # looking up 'likes' matches the label of 'gerard'
        self.assertCountEqual(
            self.kb.lookup('likes'),
            [('likes', 'object_property'), ('gerard', 'undecided')],
        )

        self.kb += ['gerard age 18']
        self.assertCountEqual(
            self.kb.lookup('age'), [('age', 'datatype_property')]
        )

    def test_literals(self):

        literals = [
            '"test"',
            '"""test"""',
            '"""toto\ntata"""',
            '"test"@fr',
            '"foo"^^<http://example.org/my/datatype>',
            '"""10"""^^xsd:decimal',
            '-5',
            '0',
            '1',
            '10',
            '+1',
            '"-5"^^xsd:integer',
            '"10"^^<http://www.w3.org/2001/XMLSchema#integer>',
            '1.3e2',
            '10e0',
            '-12.5e10',
            '"1.3e2"^^xsd:double',
            '"-12.5e10"^^<http://www.w3.org/2001/XMLSchema#double>',
            '0.0',
            '1.0',
            '1.234567890123456789',
            '-5.0',
            '"0.0"^^xsd:decimal',
            '"-5.0"^^<http://www.w3.org/2001/XMLSchema#decimal>',
            'true',
            'false',
            '"true"^^xsd:boolean',
            '"false"^^<http://www.w3.org/2001/XMLSchema#boolean>',
        ]

        malformed = ["'test'", '"toto\ntata"']

        objects = ['test', 'False', 'True']

        for i, val in enumerate(literals):
            self.kb += ['robert rel%s %s' % (i, val)]
            self.assertCountEqual(
                self.kb.lookup('rel%s' % i),
                [('rel%s' % i, 'datatype_property')],
            )

        for i, val in enumerate(objects):
            self.kb += ['robert relobj%s %s' % (i, val)]
            self.assertCountEqual(
                self.kb.lookup('relobj%s' % i),
                [('relobj%s' % i, 'object_property')],
            )

        for j, val in enumerate(malformed):
            with self.assertRaises(KbServerError):
                self.kb += ['robert relmal%s %s' % (j, val)]

    def test_retrieval(self):

        self.assertFalse(self.kb.about('Human'))
        self.assertFalse(self.kb['* rdf:type Human'])

        self.kb += ['johnny rdf:type Human', 'johnny rdfs:label "A que Johnny"']
        self.kb += ['alfred rdf:type Human', 'alfred likes icecream']

        self.assertCountEqual(
            self.kb.about('Human'),
            [
                ['johnny', 'rdf:type', 'Human'],
                ['alfred', 'rdf:type', 'Human'],
            ],
        )

        self.assertCountEqual(
            self.kb['* rdf:type Human'],
            [{'var1': 'johnny'}, {'var1': 'alfred'}],
        )

        self.kb -= ['alfred rdf:type Human', 'alfred likes icecream']

        self.assertCountEqual(
            self.kb['* rdf:type Human'], [{'var1': 'johnny'}]
        )

        self.assertTrue(self.kb['johnny rdf:type Human'])

    def test_models_retrieval(self):

        self.kb.update(['s p o', 's2 p2 o2'], ['model1', 'model2'])

        self.assertTrue(self.kb['s p o', ['model1']])
        self.assertTrue(self.kb['s2 p2 o2', ['model1']])
        self.assertTrue(self.kb['s p o', ['model2']])

    def test_complex_queries(self):
        self.assertCountEqual(
            self.kb['?agent rdf:type Robot', '?agent desires ?obj'],
            [],
        )

        self.kb += ['nono rdf:type Human', 'alfred rdf:type Robot']
        self.assertCountEqual(
            self.kb['* * *'],
            [
                {'var1': 'nono', 'var2': 'rdf:type', 'var3': 'Human'},
                {'var1': 'alfred', 'var2': 'rdf:type', 'var3': 'Robot'},
            ],
        )

        self.kb += ['nono desires jump', 'alfred desires oil']
        self.assertCountEqual(
            self.kb['* * *'],
            [
                {'var1': 'nono', 'var2': 'desires', 'var3': 'jump'},
                {'var1': 'alfred', 'var2': 'desires', 'var3': 'oil'},
                {'var1': 'nono', 'var2': 'rdf:type', 'var3': 'Human'},
                {'var1': 'alfred', 'var2': 'rdf:type', 'var3': 'Robot'},
            ],
        )

        self.kb += ['nono loves icecream']
        self.assertEqual(
            self.kb['?agent desires jump', '?agent loves icecream'],
            [{'agent': 'nono'}],
        )
        self.assertCountEqual(
            self.kb['?agent desires *', '?agent loves icecream'],
            [{'agent': 'nono'}],
        )

        self.kb += ['jump rdf:type Action']
        self.assertCountEqual(
            self.kb['?agent rdf:type Robot', '?agent desires ?obj'],
            [{'agent': 'alfred', 'obj': 'oil'}],
        )
        self.assertCountEqual(
            self.kb['?agent desires ?act', '?act rdf:type Action'],
            [{'agent': 'nono', 'act': 'jump'}],
        )
        self.assertCountEqual(
            self.kb['?agent desires ?obj'],
            [
                {'agent': 'alfred', 'obj': 'oil'},
                {'agent': 'nono', 'obj': 'jump'},
            ],
        )

    def test_update(self):
        self.kb += ['nono isNice true', 'isNice rdf:type owl:FunctionalProperty']
        self.assertCountEqual(
            self.kb['* isNice true'], [{'var1': 'nono'}]
        )

        self.kb += ['nono isNice false']
        self.assertFalse(self.kb['* isNice true'])
        self.assertCountEqual(
            self.kb['* isNice false'], [{'var1': 'nono'}]
        )

    def test_about(self):
        self.kb.update(['nono isNice true'], ['model1'])
        self.kb.update(['jamesbond isNice true'], ['model2'])
        self.assertTrue(bool(self.kb.about('nono', ['model1'])))
        self.assertFalse(bool(self.kb.about('nono', ['model2'])))

    def _get_event(self, client):
        """Get an event from the results queue, or raise Empty."""
        return self.kb.requestresults[client].get_nowait()

    def _assert_no_event(self, client):
        """Assert that no event is pending for this client."""
        if client not in self.kb.requestresults:
            return  # no queue yet means no events
        with self.assertRaises(Empty):
            self.kb.requestresults[client].get_nowait()

    def test_events(self):

        CLIENT = 'test_client'

        evt_id = self.kb.subscribe(['?o isIn room'])
        self.kb.eventsubscriptions.setdefault(evt_id, []).append(CLIENT)

        # should not trigger an event
        self.kb += ['alfred isIn garage']
        self._assert_no_event(CLIENT)

        # should trigger an event
        self.kb += ['alfred isIn room']

        msg_type, evt = self._get_event(CLIENT)
        self.assertEqual(msg_type, 'event')

        # should not trigger an event (alfred already in room)
        self.kb += ['alfred isIn room']
        self._assert_no_event(CLIENT)

        # different predicate, should not trigger
        self.kb += ['alfred leaves room']
        self._assert_no_event(CLIENT)

        # alfred is already in garage, should not fire
        evt_id2 = self.kb.subscribe(['?o isIn garage'])
        self.kb.eventsubscriptions.setdefault(evt_id2, []).append(CLIENT)
        self.assertNotEqual(evt_id, evt_id2)
        self._assert_no_event(CLIENT)

        # alfred is already in garage, should not fire
        self.kb += ['alfred isIn garage']
        self._assert_no_event(CLIENT)

        # new entity in garage, should fire
        self.kb += ['batman isIn garage']

        msg_type, evt = self._get_event(CLIENT)
        self.assertEqual(msg_type, 'event')

    def test_complex_events(self):

        CLIENT = 'test_client'

        evt_id = self.kb.subscribe(
            ['?a desires ?act', '?act rdf:type Action']
        )
        self.kb.eventsubscriptions.setdefault(evt_id, []).append(CLIENT)

        # should not trigger an event
        self.kb += ['alfred desires ragnagna']
        self._assert_no_event(CLIENT)

        # should not trigger an event
        self.kb += ['ragnagna rdf:type Zorro']
        self._assert_no_event(CLIENT)

        # should trigger an event
        self.kb += ['ragnagna rdf:type Action']

        msg_type, evt = self._get_event(CLIENT)
        self.assertEqual(msg_type, 'event')

    def test_taxonomy_walking(self):

        self.assertFalse(self.kb.classesof('john'))
        self.kb += ['john rdf:type Human']
        self.assertCountEqual(self.kb.classesof('john'), ['Human'])
        self.kb += ['john rdf:type Genius']
        self.assertCountEqual(self.kb.classesof('john'), ['Human', 'Genius'])
        self.kb -= ['john rdf:type Human']
        self.assertCountEqual(self.kb.classesof('john'), ['Genius'])

    def test_memory(self):

        self.kb += ['john rdf:type Human']
        time.sleep(0.2)
        self.assertTrue('john' in self.kb)
        self.kb -= ['john rdf:type Human']
        self.kb.update(['john rdf:type Human'], lifespan=2)
        time.sleep(0.5)
        self.kb.check_expired_stmts()
        self.assertTrue('john' in self.kb)
        time.sleep(2)
        self.kb.check_expired_stmts()
        self.assertFalse('john' in self.kb)

    def test_sparql(self):

        self.kb += ['joe eats carrot', 'ari eats electricity']

        res = self.kb.sparql('SELECT ?a WHERE { ?a :eats ?b . }')
        self.assertEqual(len(res['results']['bindings']), 2)

        # empty result
        res = self.kb.sparql('SELECT ?a WHERE { ?a :drinks ?b . }')
        self.assertEqual(len(res['results']['bindings']), 0)

        # invalid SPARQL ('eats' has no namespace prefix)
        with self.assertRaises(KbServerError):
            self.kb.sparql('SELECT ?a WHERE { ?a eats ?b . }')

    def test_label(self):

        self.kb += ['robot1 rdf:type Robot']

        # no label set -> returns the term name
        labels = self.kb.label('robot1')
        self.assertEqual(labels['default'], 'robot1')

        # set an untagged label
        self.kb += ['robot1 rdfs:label "My Robot"']
        labels = self.kb.label('robot1')
        self.assertEqual(str(labels['default']), 'My Robot')

        # set language-tagged labels
        self.kb += ['robot1 rdfs:label "Mon Robot"@fr']
        self.kb += ['robot1 rdfs:label "Mi Robot"@es']
        labels = self.kb.label('robot1')
        self.assertEqual(str(labels['fr']), 'Mon Robot')
        self.assertEqual(str(labels['es']), 'Mi Robot')

    def test_details(self):

        self.kb += [
            'Robot rdfs:subClassOf Agent',
            'ari rdf:type Robot',
            'ari likes cookies',
        ]

        details = self.kb.details('ari')
        self.assertEqual(details['id'], 'ari')
        self.assertEqual(details['type'], 'instance')
        # ari should have 'Robot' as a class
        class_ids = [
            v['id'] for v in details['attributes'][0]['values']
        ]
        self.assertIn('Robot', class_ids)
        # ari likes cookies should appear in relations
        rel_strs = [' '.join(r) for r in details['relations']]
        self.assertTrue(
            any('likes' in r and 'cookies' in r for r in rel_strs)
        )

        details = self.kb.details('Robot')
        self.assertEqual(details['type'], 'class')

    def test_wildcard_retract(self):

        self.kb += [
            'ari rdf:type Robot',
            'ari isIn kitchen',
            'tiago rdf:type Robot',
            'tiago isIn living_room',
        ]

        # retract all properties of ari
        self.kb.remove(['ari ?p ?o'])
        self.assertFalse('ari' in self.kb)

        # tiago should still be there
        self.assertTrue('tiago rdf:type Robot' in self.kb)
        self.assertTrue('tiago isIn living_room' in self.kb)

        # retract a specific predicate with wildcard object
        self.kb.remove(['tiago isIn ?loc'])
        self.assertFalse('tiago isIn living_room' in self.kb)
        self.assertTrue('tiago rdf:type Robot' in self.kb)

        # retract with wildcard subject
        self.kb.remove(['?robot rdf:type Robot'])
        self.assertFalse('tiago rdf:type Robot' in self.kb)

    def test_wildcard_retract_multi_stmt_error(self):

        self.kb += ['alpha bravo charlie', 'delta echo foxtrot']

        with self.assertRaises(KbServerError):
            self.kb.remove(['?s bravo ?o', '?s echo ?o'])

    def test_oneshot_event(self):

        CLIENT = 'test_client'

        evt_id = self.kb.subscribe(['?h rdf:type Human'], one_shot=True)
        self.kb.eventsubscriptions.setdefault(evt_id, []).append(CLIENT)

        # should trigger
        self.kb += ['joe rdf:type Human']
        msg_type, evt = self._get_event(CLIENT)
        self.assertEqual(msg_type, 'event')

        # should NOT trigger again (one-shot)
        self.kb += ['john rdf:type Human']
        self._assert_no_event(CLIENT)

    def test_event_deduplication(self):

        # subscribing the same pattern twice should return the same event id
        evt_id1 = self.kb.subscribe(['?x rdf:type Cat'])
        evt_id2 = self.kb.subscribe(['?x rdf:type Cat'])
        self.assertEqual(evt_id1, evt_id2)

    def test_model_isolation(self):

        self.kb.update(['alice rdf:type Human'], ['modelA'])
        self.kb.update(['bob rdf:type Robot'], ['modelB'])

        # alice should be visible in modelA but not modelB
        res = self.kb.find(['?s rdf:type Human'], models=['modelA'])
        self.assertEqual(len(res), 1)
        self.assertEqual(res[0]['s'], 'alice')

        res = self.kb.find(['?s rdf:type Human'], models=['modelB'])
        self.assertEqual(len(res), 0)

        # bob should be visible in modelB but not modelA
        res = self.kb.find(['?s rdf:type Robot'], models=['modelB'])
        self.assertEqual(len(res), 1)
        res = self.kb.find(['?s rdf:type Robot'], models=['modelA'])
        self.assertEqual(len(res), 0)

    def test_find_with_explicit_variables(self):

        self.kb += [
            'alice rdf:type Human',
            'bob rdf:type Human',
            'alice likes pizza',
            'bob likes pasta',
        ]

        # without restricting variables -> all named vars returned
        res = self.kb.find(
            ['?person rdf:type Human', '?person likes ?food']
        )
        self.assertTrue(all('person' in r and 'food' in r for r in res))

        # restricting to only ?food
        res = self.kb.find(
            ['?person rdf:type Human', '?person likes ?food'],
            variables=['?food'],
        )
        self.assertCountEqual(res, [{'food': 'pizza'}, {'food': 'pasta'}])
        # ?person should not be in the results
        self.assertTrue(all('person' not in r for r in res))

    def test_retract_nonexistent(self):

        # removing a statement that doesn't exist should not raise
        self.kb.remove(['nonexistent somerel something'])

    def test_exist_direct(self):

        self.kb += ['alice rdf:type Human']

        self.assertTrue(self.kb.exist(['alice rdf:type Human']))
        self.assertFalse(self.kb.exist(['bob rdf:type Human']))

        # exist with variables (acts as a query)
        self.assertTrue(self.kb.exist(['?s rdf:type Human']))
        self.assertFalse(self.kb.exist(['?s rdf:type Robot']))

        # exist with explicit models
        self.kb.update(['secret rdf:type Spy'], ['hidden_model'])
        self.assertTrue(
            self.kb.exist(['secret rdf:type Spy'], models=['hidden_model'])
        )
        self.assertFalse(
            self.kb.exist(['secret rdf:type Spy'], models=['default'])
        )

    def test_clear_keeps_defaults(self):

        self.kb += ['temporary rdf:type Thing']
        self.assertTrue('temporary rdf:type Thing' in self.kb)

        self.kb.clear(keep_defaults=True)
        # after clear, the temporary fact should be gone
        self.assertFalse('temporary' in self.kb)


def version():
    print('KnowledgeCore tests %s' % __version__)


if __name__ == '__main__':

    import argparse

    parser = argparse.ArgumentParser(
        description='Test suite for KnowledgeCore.'
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
