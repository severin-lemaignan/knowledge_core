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

import threading
import unittest

from knowledge_core.api import KB, KbError
from launch import LaunchDescription
import launch_ros
import launch_testing
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


@pytest.mark.rostest
def generate_test_description():
    kb_node = launch_ros.actions.Node(
        package='knowledge_core',
        executable='knowledge_core',
        namespace='kb',
        output='both',
        emulate_tty=True,
        arguments=['--debug', '--no-reasoner'])

    ld = LaunchDescription()
    ld.add_action(kb_node)
    ld.add_action(launch_testing.actions.ReadyToTest())
    return ld, {'kb_node': kb_node}


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, kb_node, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=kb_node)


class TestPythonicROSKb(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()
        cls.node = Node('kb_test')
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.spin_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin_thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.spin_thread.join()
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        self.kb = KB(self.node)
        self.kb.clear()

    def tearDown(self):
        pass

    def sleep(self, d):
        import time
        time.sleep(d)

    def test_basics(self):

        self.kb.hello()

        with self.assertRaises(TypeError):
            self.kb.add()
        with self.assertRaises(KbError):
            self.kb.add('toto')
        with self.assertRaises(KbError):
            self.kb.add(['toto'])
        with self.assertRaises(KbError):
            self.kb += ['toto']
        with self.assertRaises(KbError):
            self.kb += ['toto titi']
        with self.assertRaises(KbError):
            self.kb += ['toto titi tutu tata']

    def test_basic_modifications(self):

        # check no exception is raised
        self.kb.add(['johnny rdf:type Human',
                    'johnny rdfs:label "A que Johnny"'])
        self.kb += ['alfred rdf:type Human', 'alfred likes icecream']
        self.kb.remove(['alfred rdf:type Human', 'alfred likes icecream'])
        self.kb -= ['johnny rdf:type Human']

        self.kb.revise(['toto likes tata'], {'method': 'add'})

    def test_basic_model_modifications(self):

        # check no exception is raised
        self.kb.add(
            ['johnny rdf:type Human', 'johnny rdfs:label "A que Johnny"'], ['model1']
        )
        self.kb.remove(
            ['alfred rdf:type Human', 'alfred likes icecream'], ['model1'])

        self.kb.add(
            ['johnny rdf:type Human', 'johnny rdfs:label "A que Johnny"'],
            ['model1', 'model2'],
        )
        self.kb.remove(
            ['alfred rdf:type Human', 'alfred likes icecream'], ['model1', 'model2']
        )

        self.kb.revise(['toto likes tata'], {
                       'method': 'add', 'models': ['model1']})
        self.kb.revise(
            ['toto likes tata'], {'method': 'add',
                                  'models': ['model1', 'model2']}
        )

    def test_basic_kwargs(self):

        self.kb.revise(['toto likes tata'], policy={'method': 'add'})
        self.kb.add(['johnny rdf:type Human'], lifespan=10)

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
        self.assertCountEqual(self.kb.lookup('alfred'),
                              [['alfred', 'undecided']])
        self.assertCountEqual(
            self.kb.lookup('rdfs:label'), [['rdfs:label', 'datatype_property']]
        )

        self.kb += ['alfred rdf:type Robot']
        self.assertCountEqual(self.kb.lookup('alfred'),
                              [['alfred', 'instance']])
        self.assertCountEqual(self.kb.lookup('Robot'), [['Robot', 'class']])
        self.assertCountEqual(
            self.kb.lookup('rdf:type'), [['rdf:type', 'object_property']]
        )

        self.kb += ['alfred likes icecream']
        self.assertCountEqual(self.kb.lookup('likes'), [
                              ['likes', 'object_property']])
        self.assertCountEqual(self.kb.lookup('alfred'),
                              [['alfred', 'instance']])

        self.kb += ['nono rdfs:label "alfred"']
        self.kb += ['sentence rdfs:label "alfred is a charming person"']
        self.assertCountEqual(
            self.kb.lookup('alfred'),
            [['alfred', 'instance'], ['nono', 'undecided'], ['sentence', 'undecided']],
        )

        # as 'sentence' is only a partial match, it should be listed last
        self.assertEqual(
            self.kb.lookup('alfred')[-1], ['sentence', 'undecided'])

        self.kb.clear()

        # same test as above, but different insertion order
        self.kb += ['sentence rdfs:label "alfred is a charming person"']
        self.kb += ['nono rdfs:label "alfred"']
        self.kb += ['alfred likes icecream']
        self.kb += ['alfred rdf:type Robot']
        self.assertCountEqual(
            self.kb.lookup('alfred'),
            [['alfred', 'instance'], ['nono', 'undecided'], ['sentence', 'undecided']],
        )

        self.kb += ['gerard rdfs:label "likes"']
        self.assertCountEqual(
            self.kb.lookup('likes'),
            [['gerard', 'undecided'], ['likes', 'object_property']],
        )

        self.kb += ['gerard age 18']
        self.assertCountEqual(self.kb.lookup(
            'age'), [['age', 'datatype_property']])

    def test_literals(self):

        literals = {
            '"test"': 'test',
            '"""test"""': 'test',
            '"""toto\ntata"""': 'toto\ntata',
            '"test"@fr': 'test',
            '"foo"^^<http://example.org/my/datatype>': 'foo',
            '"""10"""^^xsd:decimal': 10,
            '-5': -5,
            '0': 0,
            '1': 1,
            '10': 10,
            '+1': 1,
            '"-5"^^xsd:integer': -5,
            '"10"^^<http://www.w3.org/2001/XMLSchema#integer>': 10,
            '1.3e2': 1.3e2,
            '10e0': 10,
            '-12.5e10': -12.5e10,
            '"1.3e2"^^xsd:double': 1.3e2,
            '"-12.5e10"^^<http://www.w3.org/2001/XMLSchema#double>': -12.5e10,
            '0.0': 0.0,
            '1.0': 1.0,
            '1.234567890123456789': 1.234567890123456789,
            '-5.0': -5.0,
            '"0.0"^^xsd:decimal': 0.0,
            '"-5.0"^^<http://www.w3.org/2001/XMLSchema#decimal>': -5.0,
            'true': True,
            'false': False,
            '"true"^^xsd:boolean': True,
            '"false"^^<http://www.w3.org/2001/XMLSchema#boolean>': False,
        }

        malformed = ["'test'", '"toto\ntata"']

        objects = ['test', 'False', 'True']

        for i, kv in enumerate(literals.items()):
            lit, py = kv
            self.kb += ['robert rel%s %s' % (i, lit)]
            self.assertCountEqual(
                self.kb.lookup('rel%s' % i), [
                    ['rel%s' % i, 'datatype_property']]
            )
            self.assertEqual(self.kb['* rel%s ?lit' % i][0]['lit'], py)

        for i, val in enumerate(objects):
            self.kb += ['robert relobj%s %s' % (i, val)]
            self.assertCountEqual(
                self.kb.lookup('relobj%s' % i), [
                    ['relobj%s' % i, 'object_property']]
            )

        for val in malformed:
            with self.assertRaises(KbError):
                self.kb += ['robert rel%s %s' % (i, val)]

    def test_labels(self):

        self.kb += ['johnny rdf:type Human']

        self.assertEqual(self.kb.label('johnny'), 'johnny')

        self.kb += ['johnny rdfs:label "A que Johnny"']

        self.assertEqual(self.kb.label('johnny'), 'A que Johnny')
        self.assertEqual(self.kb.label('johnny', 'en'), 'A que Johnny')
        self.assertEqual(self.kb.label('johnny', 'fr'), 'A que Johnny')
        self.assertEqual(self.kb.label('johnny', 'unknown_language'), 'A que Johnny')

        self.kb += ['johnny rdfs:label "Jojo"@fr']

        self.assertEqual(self.kb.label('johnny'), 'A que Johnny')
        self.assertEqual(self.kb.label('johnny', 'en'), 'A que Johnny')
        self.assertEqual(self.kb.label('johnny', 'fr'), 'Jojo')

    def test_retrieval(self):

        self.assertFalse(self.kb.about('Human'))
        self.assertFalse(self.kb['* rdf:type Human'])

        self.kb += ['johnny rdf:type Human',
                    'johnny rdfs:label "A que Johnny"']
        self.kb += ['alfred rdf:type Human', 'alfred likes icecream']

        self.assertCountEqual(
            self.kb.about('Human'),
            [['johnny', 'rdf:type', 'Human'], ['alfred', 'rdf:type', 'Human']],
        )

        self.assertCountEqual(
            self.kb['* rdf:type Human'], [{'var1': 'johnny'},
                                          {'var1': 'alfred'}]
        )

        self.kb -= ['alfred rdf:type Human', 'alfred likes icecream']

        self.assertCountEqual(
            self.kb['* rdf:type Human'], [{'var1': 'johnny'}])

        self.assertTrue(self.kb['johnny rdf:type Human'])

    def test_models_retrieval(self):

        self.kb.add(['s p o', 's2 p2 o2'], ['model1', 'model2'])

        self.assertTrue(self.kb['s p o', ['model1']])
        self.assertTrue(self.kb['s2 p2 o2', ['model1']])
        self.assertTrue(self.kb['s p o', ['model2']])

    def test_complex_queries(self):
        self.assertCountEqual(
            self.kb['?agent rdf:type Robot', '?agent desires ?obj'], []
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
            self.kb['?agent desires jump', '?agent loves icecream'], [
                {'agent': 'nono'}]
        )
        self.assertCountEqual(
            self.kb['?agent desires *',
                    '?agent loves icecream'], [{'agent': 'nono'}]
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
            [{'agent': 'alfred', 'obj': 'oil'}, {'agent': 'nono', 'obj': 'jump'}],
        )

    def test_update(self):
        self.kb += ['nono isNice true',
                    'isNice rdf:type owl:FunctionalProperty']
        self.assertCountEqual(self.kb['* isNice true'], [{'var1': 'nono'}])

        self.kb += ['nono isNice false']
        self.assertFalse(self.kb['* isNice true'])
        self.assertCountEqual(self.kb['* isNice false'], [{'var1': 'nono'}])

    def test_about(self):
        self.kb.add(['nono isNice true'], ['model1'])
        self.kb.add(['jamesbond isNice true'], ['model2'])
        self.assertTrue(bool(self.kb.about('nono', ['model1'])))
        self.assertFalse(bool(self.kb.about('nono', ['model2'])))

    def test_events(self):

        eventtriggered = [False]

        def onevent(evt):
            # py3 only! -> workaround is turning eventtriggered into a list
            # nonlocal eventtriggered
            print('In callback. Got evt %s' % evt)
            eventtriggered[0] = True

        evtid = self.kb.subscribe(['?o isIn room'], onevent)

        # should not trigger an event
        self.kb += ['alfred isIn garage']
        self.sleep(0.5)
        self.assertFalse(eventtriggered[0])

        # should trigger an event
        self.kb += ['alfred isIn room']
        self.sleep(0.5)
        self.assertTrue(eventtriggered[0])

        eventtriggered[0] = False

        # should not trigger an event
        self.kb += ['alfred leaves room']
        self.sleep(0.5)
        self.assertFalse(eventtriggered[0])

        # alfred is already in garage, should not fire an event
        evtid2 = self.kb.subscribe(['?o isIn garage'], onevent)
        self.assertFalse(evtid == evtid2)
        self.sleep(0.5)
        self.assertFalse(eventtriggered[0])

        # alfred is already in garage, should not fire an event
        self.kb += ['alfred isIn garage']
        self.sleep(0.5)
        self.assertFalse(eventtriggered[0])

        self.kb += ['batman isIn garage']
        self.sleep(0.5)
        self.assertTrue(eventtriggered[0])

    def test_events_multi_clients(self):

        evt_triggers_count = [0]
        evt_triggers_count_other = [0]

        def onevent(evt):
            print('onevent callback triggered')
            evt_triggers_count[0] += 1

        def onevent_other(evt):
            print('onevent_other callback triggered')
            evt_triggers_count_other[0] += 1

        evtid = self.kb.subscribe(['?o isIn room'], onevent)

        evtid2 = self.kb.subscribe(['?o isIn room'], onevent)

        self.assertEqual(evtid, evtid2)

        # should trigger only one event
        self.kb += ['alfred isIn room']
        self.sleep(0.5)
        self.assertEqual(evt_triggers_count[0], 1)

        evt_triggers_count = [0]

        ##########################################################
        evtid2 = self.kb.subscribe(['?o isIn room'], onevent_other)

        # each callback should be called once
        self.kb += ['catwoman isIn room']
        self.sleep(0.5)
        self.assertEqual(evt_triggers_count[0], 1)
        self.assertEqual(evt_triggers_count_other[0], 1)

        evt_triggers_count = [0]
        evt_triggers_count_other = [0]

        ##########################################################
        kb2 = KB(self.node)
        evtid3 = kb2.subscribe(['?o isIn room'], onevent)
        self.assertEqual(evtid, evtid3)

        # should trigger two event (two independent instances of KB(), that
        # happen to call the same callback)
        self.kb += ['batman isIn room']
        self.sleep(0.5)
        self.assertEqual(evt_triggers_count[0], 2)

        evt_triggers_count = [0]

        evtid3 = kb2.subscribe(['?o isIn room'], lambda x: x)

        # should again trigger two event (two independent instances of KB(), that
        # happen to call the same callback)
        #
        # adding another event listener should not cause the callbacks to be
        # called yet another time
        self.kb += ['wonderwoman isIn room']
        self.sleep(0.5)
        self.assertEqual(evt_triggers_count[0], 2)

    def test_complex_events(self):

        eventtriggered = [False]
        last_evt = []

        def onevent(evt):
            self.node.get_logger().warn('In callback. Got evt %s' % evt)
            eventtriggered[0] = True
            last_evt.append(evt)

        self.kb.subscribe(
            ['?a desires ?act', '?act rdf:type Action'], onevent)

        # should not trigger an event
        self.kb += ['alfred desires ragnagna']
        self.sleep(0.2)

        self.assertFalse(eventtriggered[0])

        # should not trigger an event
        self.kb += ['ragnagna rdf:type Zorro']
        self.sleep(0.2)

        self.assertFalse(eventtriggered[0])

        # should trigger an event
        self.kb += ['ragnagna rdf:type Action']
        self.sleep(0.2)

        self.assertTrue(eventtriggered[0])
        self.assertEqual(last_evt[-1], [{'a': 'alfred', 'act': 'ragnagna'}])

    # test currently disabled, as taxonomy walking is not yet supported over
    # the ROS API
    def _test_taxonomy_walking(self):

        self.assertFalse(self.kb.classesof('john'))
        self.kb += ['john rdf:type Human']
        self.assertCountEqual(self.kb.classesof('john'), ['Human'])
        self.kb += ['john rdf:type Genius']
        self.assertCountEqual(self.kb.classesof('john'), ['Human', 'Genius'])
        self.kb -= ['john rdf:type Human']
        self.assertCountEqual(self.kb.classesof('john'), ['Genius'])
