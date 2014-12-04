
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import logging
import unittest
import time
import kb
from minimalkb import __version__

from Queue import Empty

REASONING_DELAY = 0.2

class TestSequenceFunctions(unittest.TestCase):

	def setUp(self):
	        self.kb = kb.KB()
	        self.kb.clear()

	def tearDown(self):
        	self.kb.close()

	def test_reason(self):

		# basic adds :

		self.kb += ['self rdf:type robot']
        	
        	self.kb.add(['robot rdfs:subClassOf agent', 'agent rdfs:subClassOf humanoide', 'human rdfs:subClassOf animals'])

	        self.kb.add(['robot owl:equivalentClass machine', 'machine owl:equivalentClass automate'])
	
		self.kb += ['rp2 rdfs:subClassOf automate']
		
		# basic tests :

		self.assertTrue('self rdf:type robot' in self.kb)

		time.sleep(0.1)

		self.assertTrue('robot rdfs:subClassOf humanoide' in self.kb)
		self.assertTrue('self rdf:type humanoide' in self.kb)
		self.assertTrue('robot owl:equivalentClass automate' in self.kb)
		self.assertTrue('self rdf:type machine' in self.kb)
		self.assertTrue('self rdf:type automate' in self.kb)

		self.assertTrue('rp2 rdfs:subClassOf robot' in self.kb)

		


def version():
    print("minimalKB tests %s" % __version__)

if __name__ == '__main__':

    import argparse

    parser = argparse.ArgumentParser(description='Test suite for minimalKB.')
    parser.add_argument('-v', '--version', action='version',
                       version=version(), help='returns minimalKB version')
    parser.add_argument('-f', '--failfast', action='store_true',
                                help='stops at first failed test')

    args = parser.parse_args()

    kblogger = logging.getLogger("kb")
    console = logging.StreamHandler()
    kblogger.setLevel(logging.DEBUG)
    kblogger.addHandler(console)

    unittest.main(failfast=args.failfast)

