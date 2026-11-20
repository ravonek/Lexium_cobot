%% trentthesis.cls
%% Copyright (c) 2026 Md Robiuddin. Released under the MIT licence (see LICENSE).
%% Unofficial community thesis template.
%% LaTeX class for a University thesis.
%% It reproduces the look of the Quarto-based thesis template (Thesis.qmd and
%% custom_heading.tex) . Formatting guidelines:
%%   12 pt text, A4 paper (changed from letter for KBTU), 38 mm left margin and 25 mm elsewhere, double-spaced body,
%%   page numbers centred at the bottom, no running chapter titles,
%%   20/16/14/12 pt bold ragged-right headings, captions beneath figures and tables.
\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{trentthesis}[2026/10/01 v1.0 University undergraduate thesis]

\DeclareOption*{\PassOptionsToClass{\CurrentOption}{report}}
\ProcessOptions\relax
\LoadClass[12pt,a4paper,oneside]{report}% A4 for KBTU (the Trent original is letter paper)

\RequirePackage[T1]{fontenc}
\RequirePackage[utf8]{inputenc}
\RequirePackage{lmodern}
\RequirePackage[left=38mm,right=25mm,top=25mm,bottom=25mm,footskip=12.5mm]{geometry}
\RequirePackage{microtype}
\RequirePackage{setspace}
\RequirePackage{graphicx}
\RequirePackage{adjustbox}
\RequirePackage{booktabs,tabularx,array,multirow}
\RequirePackage{amsmath,amssymb}
\RequirePackage{enumitem}
\RequirePackage[numbers,sort&compress]{natbib}
\RequirePackage[font=small,labelfont=bf,skip=8pt,position=bottom]{caption}
\RequirePackage{titlesec}
\RequirePackage[titles]{tocloft}
\RequirePackage{placeins}
\RequirePackage[hidelinks,pdfusetitle]{hyperref}
\RequirePackage{bookmark}

%% ---------------------------------------------------------------- metadata
%% Set these in main.tex. \title, \author and \date are the standard commands.
\title{Design and Development of a Robotic System}
\author{Rakhim Zhan}
\date{Month Year}
\newcommand{\program}[1]{\gdef\@program{#1}}
\newcommand{\degree}[1]{\gdef\@degree{#1}}
\newcommand{\copyrightyear}[1]{\gdef\@copyrightyear{#1}}
\program{Department Name}
\degree{Degree Name}
\copyrightyear{\the\year}
% Read-only aliases, handy in the abstract page.
\newcommand{\thesistitle}{\@title}
\newcommand{\thesisauthor}{\@author}

%% ---------------------------------------------------------------- layout
\setlength{\parskip}{0pt}
\setlength{\emergencystretch}{2em}
\clubpenalty=10000
\widowpenalty=10000
\displaywidowpenalty=10000
\setcounter{secnumdepth}{2}
\setcounter{tocdepth}{2}
\setlist{itemsep=3pt,topsep=6pt}
\renewcommand{\arraystretch}{1.16}
\pagestyle{plain}% page number centred at the bottom, no running titles

%% Headings as in custom_heading.tex: 20/16/14/12 pt bold, ragged right, no hyphenation.
%% A chapter here is a "# Heading" of the Quarto template.
\titleformat{\chapter}[hang]
  {\normalfont\fontsize{20pt}{24pt}\bfseries\raggedright\hyphenpenalty=10000\exhyphenpenalty=10000}
  {\thechapter}{1em}{}
\titleformat{name=\chapter,numberless}[hang]
  {\normalfont\fontsize{20pt}{24pt}\bfseries\raggedright\hyphenpenalty=10000\exhyphenpenalty=10000}
  {}{0pt}{}
\titlespacing*{\chapter}{0pt}{0pt}{24pt}
\titleformat{\section}
  {\normalfont\fontsize{16pt}{20pt}\bfseries\raggedright\hyphenpenalty=10000\exhyphenpenalty=10000}
  {\thesection}{1em}{}
\titleformat{\subsection}
  {\normalfont\fontsize{14pt}{18pt}\bfseries\raggedright}
  {\thesubsection}{1em}{}
\titleformat{\subsubsection}
  {\normalfont\fontsize{12pt}{16pt}\bfseries\raggedright}
  {\thesubsubsection}{1em}{}
\titleformat{\paragraph}
  {\normalfont\fontsize{12pt}{16pt}\bfseries\raggedright}
  {\theparagraph}{1em}{}
\titlespacing*{\paragraph}{0pt}{3.25ex plus 1ex minus .2ex}{1ex plus .2ex}

%% Contents and lists, with "Table"/"Figure" before each number as in the template.
\renewcommand{\contentsname}{Table of Contents}
\renewcommand{\listfigurename}{List of Figures}
\renewcommand{\listtablename}{List of Tables}
\renewcommand{\bibname}{Bibliography}
\renewcommand{\cftfigpresnum}{\figurename~}
\renewcommand{\cfttabpresnum}{\tablename~}
\setlength{\cftfignumwidth}{5.6em}
\setlength{\cfttabnumwidth}{5.2em}

%% ---------------------------------------------------------------- front matter
% An unnumbered chapter that is still listed in the Table of Contents.
\newcommand{\frontchapter}[1]{%
  \clearpage\phantomsection
  \addcontentsline{toc}{chapter}{#1}%
  \chapter*{#1}}

% Title page: counted as page i, number not shown. Leaves the counter at 2.
\newcommand{\maketitlepage}{%
  \hypersetup{pageanchor=false}%
  \pagenumbering{roman}%
  \begin{titlepage}%
    \centering
    \setstretch{1.25}%
    \vspace*{10mm}%
    {\sffamily\LARGE\bfseries\@title\par}%
    \vspace{20mm}%
    by\par
    \@author\par
    \vfill
    A THESIS\\
    SUBMITTED TO THE PROGRAM OF\\
    \MakeUppercase{\@program}\\
    AND THE FACULTY OF GRADUATE STUDIES\\
    OF ABC UNIVERSITY\\
    IN PARTIAL FULFILLMENT OF THE REQUIREMENTS\\
    FOR THE DEGREE OF\\
    \textbf{\MakeUppercase{\@degree}}\par
    \vfill
    \textcopyright~Copyright \@copyrightyear{} by \@author\\
    ABC University\\
    59 Tole Bi Street, Almaty-050000, Kazakhstan\\
    \@date\par
  \end{titlepage}%
  \setcounter{page}{2}%
  \hypersetup{pageanchor=true}}

% Supervisory committee page (page ii). Put \committeemember lines inside the environment.
\newenvironment{supervisorycommittee}{%
  \clearpage
  {\setstretch{1}\noindent\normalfont\fontsize{20pt}{24pt}\bfseries\raggedright\@title\par}%
  \vfill
  \begin{center}
  by\par
  \@author\par
  \end{center}
  \vfill
  \phantomsection
  \addcontentsline{toc}{chapter}{Supervisory Committee}%
  \noindent\textbf{Supervisory Committee}
  \begin{flushleft}
  \setstretch{1.25}}{%
  \end{flushleft}}
\newcommand{\committeemember}[3]{%
  \bigskip
  \rule{\linewidth}{1pt}\\
  #1,\\
  \underline{#2}\\
  (\textit{#3})\par}

% Abstract page: title and author above the text, as the guidelines ask.
% A Master's abstract is limited to 150 words and a PhD abstract to 350.
\newenvironment{thesisabstract}{%
  \frontchapter{Abstract}%
  \begin{center}
  \@title\par
  \@author\par
  \end{center}}{\par}
\newcommand{\keywords}[1]{\par\bigskip\noindent\textbf{Keywords:} #1\par}

%% ---------------------------------------------------------------- back matter
% Bibliography entry in the Table of Contents; use it just before \bibliography.
\newcommand{\bibliographyheading}{%
  \clearpage
  \singlespacing
  \phantomsection
  \addcontentsline{toc}{chapter}{\bibname}}

\endinput
