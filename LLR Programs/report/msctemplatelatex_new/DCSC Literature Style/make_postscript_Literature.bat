latex.exe --src -interaction=nonstopmode mscLiterature.tex
bibtex.exe mscLiterature
latex.exe --src -interaction=nonstopmode mscLiterature.tex
makeindex.exe mscLiterature.nlo -s StyleStuff/mynomencl.ist -o mscLiterature.nls
latex.exe --src -interaction=nonstopmode mscLiterature
makeindex.exe -s StyleStuff/myindex.ist mscLiterature.idx
latex.exe --src -interaction=nonstopmode mscLiterature
bibtex.exe mscLiterature
latex.exe --src -interaction=nonstopmode mscLiterature
REM Convert DVI to Postscript:
dvips.exe -P pdf -j0 mscLiterature.dvi
REM Convert Postscript to PDF:
ps2pdf.exe -sPAPERSIZE#a4 -dSAFER -dBATCH -dNOPAUSE -sDEVICE#pdfwrite -dColorConversionStrategy#/LeaveColorUnchanged -dPDFSETTINGS#/printer "mscLiterature.ps" "mscLiterature.pdf"
pause
