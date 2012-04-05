"""
    tutorialformatter
    ===========================

    This extension provides a directive to include a source code file
    in a document, but with certain comments from the file formatted
    as regular document text.  This allows code for a tutorial to look like:

        /// BEGIN_TUTORIAL
        /// This next line adds one.
        i = i + 1;
        /// Then we need to double it.
        i = i * 2;
        /// END_TUTORIAL

    And have it formatted as

    This next line adds one.::
        i = i + 1;

    Then we need to double it.::
        i = i * 2;

    The special-looking comment character sequence at the start of
    each text line can be anything not starting or ending with
    whitespace.  tutorialformatter starts by scanning the file for the
    string BEGIN_TUTORIAL.  When it finds it, it takes all the
    characters before BEGIN_TUTORIAL on that line, strips whitespace
    from the left, and uses that as the text marker.  So this would
    also be fine:

        #My Tutorial# BEGIN_TUTORIAL
        #My Tutorial# This next line adds one.
        i = i + 1
        #My Tutorial# Then we need to double it.
        i = i * 2
        #My Tutorial# END_TUTORIAL

    .. moduleauthor::  Dave Hershberger <hersh@willowgarage.com>
"""

__version__ = '0.1.0'

import os
from docutils.parsers import rst
from docutils.parsers.rst.directives import flag, unchanged
from docutils.statemachine import string2lines
from pygments.lexers import get_lexer_for_filename

class TutorialFormatterDirective(rst.Directive):
    has_content = False
    final_argument_whitespace = True
    required_arguments = 1

    option_spec = dict(shell=flag, prompt=flag, nostderr=flag,
                       in_srcdir=flag, extraargs=unchanged,
                       until=unchanged)

    def run(self):
        filename = self.arguments[0]
        text_tag = None
        tag_len = 0

        filepath = self.state.document.settings.env.srcdir
        absfilename = os.path.join( filepath, filename )
        if absfilename.endswith('.h'):
            language = 'c++'
        elif absfilename.endswith('CMakeLists.txt'):
            language = 'cmake'
        else:
            try:
                language = get_lexer_for_filename( absfilename ).name.lower()
                if language == 'text only':
                    language = 'none'
            except:
                language = 'none'
        code_prefix = '\n.. code-block:: ' + language + '\n\n'
        code_suffix = '\n'

        print "tutorial-formatter running on", absfilename
        file_ = open( absfilename, 'r' )
        text_to_process = ""
        current_block = ""
        in_code = False
        in_text = False
        in_tutorial = False
        for line in file_:
            if not in_tutorial:
                begin_pos = line.find( 'BEGIN_TUTORIAL' )
                if begin_pos != -1:
                    text_tag = line[:begin_pos].lstrip()
                    tag_len = len( text_tag )
                    in_tutorial = True
                continue
            if line.find( 'END_TUTORIAL' ) != -1:
                break
            stripped = line.lstrip()
            if stripped.startswith( text_tag.strip() ):
                if in_code:
                    text_to_process += code_prefix + current_block + code_suffix
                    current_block = ""
                    in_code = False
                in_text = True
                addition = stripped[tag_len:]
                if addition == '' or addition[-1] != '\n':
                    addition += '\n'
                current_block += addition
            else:
                if in_text:
                    text_to_process += current_block
                    current_block = ""
                    in_text = False
                    in_code = True # Code to show begins right after tagged text
                if in_code:
                    current_block += ' ' + line
        if in_code:
            text_to_process += code_prefix + current_block + code_suffix
        elif in_text:
            text_to_process += current_block

        # Debug writes...
        #print 'text_to_process ='
        #print text_to_process
        #print '= text_to_process'

        lines = string2lines( text_to_process )
        self.state_machine.insert_input( lines, absfilename )

        return []

def setup(app):
    app.add_directive('tutorial-formatter', TutorialFormatterDirective)
