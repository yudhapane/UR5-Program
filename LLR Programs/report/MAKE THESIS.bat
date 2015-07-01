latex.exe --src -interaction=nonstopmode mscThesisThijsRamakers.tex
bibtex.exe mscThesisThijsRamakers
latex.exe --src -interaction=nonstopmode mscThesisThijsRamakers.tex
makeindex.exe mscThesisThijsRamakers.nlo -s StyleStuff/mynomencl.ist -o mscThesisThijsRamakers.nls
latex.exe --src -interaction=nonstopmode mscThesisThijsRamakers
makeindex.exe -s StyleStuff/myindex.ist mscThesisThijsRamakers.idx
latex.exe --src -interaction=nonstopmode mscThesisThijsRamakers
bibtex.exe mscThesisThijsRamakers
latex.exe --src -interaction=nonstopmode mscThesisThijsRamakers
REM Convert DVI to Postscript:
dvips.exe -P pdf -j0 mscThesisThijsRamakers.dvi
REM Convert Postscript to PDF:
ps2pdf.exe -sPAPERSIZE#a4 -dSAFER -dBATCH -dNOPAUSE -sDEVICE#pdfwrite -dColorConversionStrategy#/LeaveColorUnchanged -dPDFSETTINGS#/printer "mscThesisThijsRamakers.ps" "mscThesisThijsRamakers.pdf"
pause
