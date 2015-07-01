These are the latex files for creating MSc Literature and Thesis reports
using the novel TU Delft house style.


These files originally implemented style classes for LaTeX master
thesis reports at the faculty of Aerospace engineering of the Delft University of Technology.

The files have been changed subsequently by MSc students and
staff members of the Delft Center for Systems and Control.

The most recent version of the style file can be found
at the DCSC website on the page
http://www.dcsc.tudelft.nl/Education/index.html
or direcly as
http://www.dcsc.tudelft.nl/Education/MScTemplateLatex_new.zip

If you find errors in these files or if you have corrected
bugs, please contact the DCSC MSc coordinator Peter Heuberger
so that the templates can be updated.


There are 2 styles:
1. In the subdirectory <DCSC Literature Style>:
   for a literature study with style file mscLiterature.cls
   example file mscLiterature.tex 
   To compile this file (windows only), two DOS command file have been added:
   - make_bitmap_Literature.bat       (creates pdf directly from dvi files, no eps graphics allowed)
   - make_postscript_Literature.bat (creates pdf files via postcript, eps graphics allowed)
   Just run one of these files and a file mscLiterature.pdf will result
   all auxiliary (temporary files) are removed with runnng the file mrproperLatex.bat
2. In the subdirectory <DCSC Thesis Style>
   for an MSc thesis with style file mscThesis.cls
   example fil mscThesis.tex 
   To compile this file (windows only), two DOS command file have been added:
   - make_bitmap_Thesis.bat       (creates pdf directly from dvi files, no eps graphics allowed)
   - make_postscript_Thesis.bat (creates pdf files via postcript, eps graphics allowed)
   Just run one of these files and a file mscLiterature.pdf will result
   all auxiliary (temporary files) are removed with runnng the file mrproperLatex.bat

NOTE: The first page (Titlepage) is not be changed with respect to lay-out. Only the picture on the titlepage is optional 

Last updated: October 20, 2009
By Steven Mulder and Peter Heuberger