#!/usr/bin/env python

#
# Generated Thu Mar  8 22:40:24 2012 by generateDS.py version 2.7b.
#

import sys

import ParameterDatabase as supermod

etree_ = None
Verbose_import_ = False
(   XMLParser_import_none, XMLParser_import_lxml,
    XMLParser_import_elementtree
    ) = range(3)
XMLParser_import_library = None
try:
    # lxml
    from lxml import etree as etree_
    XMLParser_import_library = XMLParser_import_lxml
    if Verbose_import_:
        print("running with lxml.etree")
except ImportError:
    try:
        # cElementTree from Python 2.5+
        import xml.etree.cElementTree as etree_
        XMLParser_import_library = XMLParser_import_elementtree
        if Verbose_import_:
            print("running with cElementTree on Python 2.5+")
    except ImportError:
        try:
            # ElementTree from Python 2.5+
            import xml.etree.ElementTree as etree_
            XMLParser_import_library = XMLParser_import_elementtree
            if Verbose_import_:
                print("running with ElementTree on Python 2.5+")
        except ImportError:
            try:
                # normal cElementTree install
                import cElementTree as etree_
                XMLParser_import_library = XMLParser_import_elementtree
                if Verbose_import_:
                    print("running with cElementTree")
            except ImportError:
                try:
                    # normal ElementTree install
                    import elementtree.ElementTree as etree_
                    XMLParser_import_library = XMLParser_import_elementtree
                    if Verbose_import_:
                        print("running with ElementTree")
                except ImportError:
                    raise ImportError("Failed to import ElementTree from any known place")

def parsexml_(*args, **kwargs):
    if (XMLParser_import_library == XMLParser_import_lxml and
        'parser' not in kwargs):
        # Use the lxml ElementTree compatible parser so that, e.g.,
        #   we ignore comments.
        kwargs['parser'] = etree_.ETCompatXMLParser()
    doc = etree_.parse(*args, **kwargs)
    return doc

#
# Globals
#

ExternalEncoding = 'ascii'

#
# Data representation classes
#

class SerialisationFlagsSub(supermod.SerialisationFlags):
    def __init__(self, serialisationFlag=None):
        super(SerialisationFlagsSub, self).__init__(serialisationFlag, )
supermod.SerialisationFlags.subclass = SerialisationFlagsSub
# end class SerialisationFlagsSub


class udb_typeSub(supermod.udb_type):
    def __init__(self, typeName=None, sendFunction=None, setFunction=None, mavlinkType=None):
        super(udb_typeSub, self).__init__(typeName, sendFunction, setFunction, mavlinkType, )
supermod.udb_type.subclass = udb_typeSub
# end class udb_typeSub


class ParameterSub(supermod.Parameter):
    def __init__(self, parameterName=None, udb_param_type=None, variable_name='NULL', description='no description', min='0.0', max='0.0', readonly=True):
        super(ParameterSub, self).__init__(parameterName, udb_param_type, variable_name, description, min, max, readonly, )
supermod.Parameter.subclass = ParameterSub
# end class ParameterSub


class ParametersSub(supermod.Parameters):
    def __init__(self, parameter=None):
        super(ParametersSub, self).__init__(parameter, )
supermod.Parameters.subclass = ParametersSub
# end class ParametersSub


class ParameterBlockSub(supermod.ParameterBlock):
    def __init__(self, blockName=None, storage_area='None', serialisationFlags=None, load_callback=None, in_mavlink_parameters=None, parameters=None, description='no description'):
        super(ParameterBlockSub, self).__init__(blockName, storage_area, serialisationFlags, load_callback, in_mavlink_parameters, parameters, description, )
supermod.ParameterBlock.subclass = ParameterBlockSub
# end class ParameterBlockSub


class ParameterBlocksSub(supermod.ParameterBlocks):
    def __init__(self, parameterBlock=None):
        super(ParameterBlocksSub, self).__init__(parameterBlock, )
supermod.ParameterBlocks.subclass = ParameterBlocksSub
# end class ParameterBlocksSub


class ParameterDatabaseSub(supermod.ParameterDatabase):
    def __init__(self, serialisationFlags=None, parameterBlocks=None):
        super(ParameterDatabaseSub, self).__init__(serialisationFlags, parameterBlocks, )
supermod.ParameterDatabase.subclass = ParameterDatabaseSub
# end class ParameterDatabaseSub



def get_root_tag(node):
    tag = supermod.Tag_pattern_.match(node.tag).groups()[-1]
    rootClass = None
    if hasattr(supermod, tag):
        rootClass = getattr(supermod, tag)
    return tag, rootClass


def parse(inFilename):
    doc = parsexml_(inFilename)
    rootNode = doc.getroot()
    rootTag, rootClass = get_root_tag(rootNode)
    if rootClass is None:
        rootTag = 'SerialisationFlags'
        rootClass = supermod.ParameterDatabase
    rootObj = rootClass.factory()
    rootObj.build(rootNode)
    # Enable Python to collect the space used by the DOM.
    doc = None
##     sys.stdout.write('<?xml version="1.0" ?>\n')
##     rootObj.export(sys.stdout, 0, name_=rootTag,
##         namespacedef_='')
    doc = None
    return rootObj


def parseString(inString):
    from StringIO import StringIO
    doc = parsexml_(StringIO(inString))
    rootNode = doc.getroot()
    rootTag, rootClass = get_root_tag(rootNode)
    if rootClass is None:
        rootTag = 'SerialisationFlags'
        rootClass = supermod.ParameterDatabase
    rootObj = rootClass.factory()
    rootObj.build(rootNode)
    # Enable Python to collect the space used by the DOM.
    doc = None
##     sys.stdout.write('<?xml version="1.0" ?>\n')
##     rootObj.export(sys.stdout, 0, name_=rootTag,
##         namespacedef_='')
    return rootObj


def parseLiteral(inFilename):
    doc = parsexml_(inFilename)
    rootNode = doc.getroot()
    rootTag, rootClass = get_root_tag(rootNode)
    if rootClass is None:
        rootTag = 'SerialisationFlags'
        rootClass = supermod.ParameterDatabase
    rootObj = rootClass.factory()
    rootObj.build(rootNode)
    # Enable Python to collect the space used by the DOM.
    doc = None
##     sys.stdout.write('#from ParameterDatabase import *\n\n')
##     sys.stdout.write('import ParameterDatabase as model_\n\n')
##     sys.stdout.write('rootObj = model_.SerialisationFlags(\n')
##     rootObj.exportLiteral(sys.stdout, 0, name_="SerialisationFlags")
##     sys.stdout.write(')\n')
    return rootObj


USAGE_TEXT = """
Usage: python ???.py <infilename>
"""

def usage():
    print USAGE_TEXT
    sys.exit(1)


def main():
    args = sys.argv[1:]
    if len(args) != 1:
        usage()
    infilename = args[0]
    root = parse(infilename)


if __name__ == '__main__':
    #import pdb; pdb.set_trace()
    main()


