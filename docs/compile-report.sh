#!/bin/bash
mkdir -p latex-build
lualatex -output-directory=latex-build lab-report.tex 
mv latex-build/lab-report.pdf .
rm -rf latex-build
