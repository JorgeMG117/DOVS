#!/bin/bash

pdflatex "main.tex"
makeindex "main.idx"
pdflatex "main.tex"