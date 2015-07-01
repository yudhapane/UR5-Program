latex.exe --src -interaction=nonstopmode mscThesis.tex
bibtex.exe mscThesis
latex.exe --src -interaction=nonstopmode mscThesis.tex
makeindex.exe mscThesis.nlo -s StyleStuff/mynomencl.ist -o mscThesis.nls
latex.exe --src -interaction=nonstopmode mscThesis
makeindex.exe -s StyleStuff/myindex.ist mscThesis.idx
latex.exe --src -interaction=nonstopmode mscThesis
bibtex.exe mscThesis
REM Convert DVI straight to PDF:
pdflatex.exe -interaction=nonstopmode mscThesis
pause
