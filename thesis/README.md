# University thesis template for Overleaf

An unofficial LaTeX thesis template for Trent University graduate students. It is a LaTeX version of the community Quarto template and follows the [Trent School of Graduate Studies 2025 formatting guidelines](https://www.trentu.ca/graduatestudies/sites/trentu.ca.graduatestudies/files/documents/Thesis%20formatting%20guidelines_FINAL_2025.pdf): 12 pt text, letter paper, 38 mm left margin and 25 mm elsewhere, double-spaced body, centred page numbers, no running titles, 20/16/14/12 pt bold headings, and IEEE references.

This template is not issued or endorsed by Trent University. Its layout is a reading of the guidelines, so check your final PDF against the current guidelines and with the Graduate Studies office before you submit.

## Use it

1. In Overleaf choose **New Project, Upload Project** and select the zip file.
2. Open `main.tex`. The compiler is pdfLaTeX (Overleaf's default), and BibTeX runs automatically.
3. In `main.tex` set your title, name, department, degree, copyright year and convocation date, then fill in the supervisory committee.
4. Write the abstract in `frontmatter/abstract.tex` (150 words for a Master's thesis, 350 for a PhD) and the acknowledgements in `frontmatter/acknowledgements.tex`. Delete the `\input{frontmatter/acknowledgements}` line in `main.tex` if you do not want that page.
5. Replace the example chapters in `chapters/` with your own and update the `\include` lines in `main.tex`. Chapters 3 and 4 show how to cite, write equations, build tables, add figures and plots, and cross-reference. Delete them when you no longer need them.
6. Put images in `figures/` and sources in `references.bib`.

## KBTU starting pages

Everything before the Introduction is set with the KBTU layout from `kbtufront.sty`: A4, Times (the `tempora` font, which also has the Cyrillic and Kazakh letters), 14 pt, single spacing, margins 30/15/20/20 mm, as in the KBTU attestation rules (section 9.1) and the 6B07114 reference diploma. The pages are, in order:

1. title page (page 1, number not printed),
2. diploma project assignment, then consultations, work schedule and signatures (`frontmatter/assignment.tex`),
3. project sheet (`frontmatter/project_sheet.tex`); the page count of the explanatory note is filled in by LaTeX,
4. АҢДАТПА, ABSTRACT and АННОТАЦИЯ (`frontmatter/abstract_kk.tex`, `abstract.tex`, `abstract_ru.tex`),
5. CONTENT, built by LaTeX from the chapter headings,
6. LIST OF ABBREVIATIONS AND SYMBOLS (`frontmatter/abbreviations.tex`).

Names, the programme, the year and the project title are set once in `main.tex`. Pages are numbered in arabic from the title page on, and the chapters carry on the count. The paper size of the whole document is A4. The chapters keep the template's own layout (12 pt Latin Modern, double-spaced, 38 mm left margin), which differs from KBTU section 9.1.

## Files

| File | Purpose |
|---|---|
| `main.tex` | Your details, front matter order, list of chapters, bibliography |
| `trentthesis.cls` | The layout. You normally do not edit it |
| `kbtufront.sty` | Layout of the KBTU starting pages |
| `frontmatter/` | Assignment, project sheet, the three abstracts, abbreviations |
| `chapters/` | One file per chapter |
| `figures/` | Images |
| `references.bib` | Sources, cited with `\cite{key}` |

## Layout notes

- Page order follows the Quarto template: title page (i, number hidden), supervisory committee, abstract, acknowledgements, Table of Contents, List of Tables, List of Figures, then the chapters in arabic numerals.
- Pages ii and iii are set at 1.25 line spacing and everything from the acknowledgements on is double-spaced, as in the Quarto template. Tables, figure captions, the contents lists and the bibliography are single-spaced.
- Captions go beneath figures and tables. Put `\caption` after the `tabular` and `\label` after the `\caption`.
- Numbering is per chapter (Figure 3.1, Table 3.2, Equation 3.4).

## Credits and licence

The page order, heading sizes and margins follow the Quarto-based [Trent_University-Thesis-Template](https://github.com/HamzaGiTX786/Trent_University-Thesis-Template) by Hamza Attarwala. This LaTeX version is a separate implementation, and no files were copied from that repository.

Copyright (c) 2026 Md Robiuddin. Released under the MIT licence; see `LICENSE`.
