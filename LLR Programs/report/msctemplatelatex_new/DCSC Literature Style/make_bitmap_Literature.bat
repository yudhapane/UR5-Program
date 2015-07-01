latex.exe --src -interaction=nonstopmode mscLiterature.tex
bibtex.exe mscLiterature
latex.exe --src -interaction=nonstopmode mscLiterature.tex
makeindex.exe mscLiterature.nlo -s StyleStuff/mynomencl.ist -o mscLiterature.nls
latex.exe --src -interaction=nonstopmode mscLiterature
makeindex.exe -s StyleStuff/myindex.ist mscLiterature.idx
latex.exe --src -interaction=nonstopmode mscLiterature
bibtex.exe mscLiterature
REM Convert DVI straight to PDF:
pdflatex.exe -interaction=nonstopmode mscLiterature
pause
