BRS manuscript sources

Compile main.tex with XeLaTeX or Tectonic. IEEEtran.cls and IEEEtran.bst are supplied. Install genuine Times New Roman fonts. prepare_fonts.ps1 can locate local Windows fonts; fonts are not redistributed in this archive.

The active source is main.tex plus sections/, the referenced analysis/*.tex files, and figures/. Chanwo_TVT_Manuscript.pdf is the compiled reading copy. Captions use at most 20 words (Figure 1 at most 15), except Figure 8, whose additional sentence specifies the task set and timing study.

Native C++ experiment code and all 100 tasks are available at https://github.com/libai1943/MiningPocket . The fixed comparison, component, and candidate-selection records in inputs/native_study/results/ reproduce the manuscript statistics. Exported C++ geometry in inputs/native/ reproduces the actual plotted constructions.

To regenerate: install numpy, matplotlib, reportlab, pymupdf, pillow, and pypdf; run analyze_native.py, draw_architecture.py --font-dir <Times-New-Roman-font-directory>, and make_native_figures.py, then compile main.tex. finish_build.ps1 combines these steps on Windows. Set MININGPOCKET_ROOT to an external native project root to use its completed study records instead of the included records. The scripts do not rerun planning during paper generation.

Author order: Bai Li, Fan Bu, Guizhen Yu, Zhijiang Shao. Funding and the final submission declarations are to be supplied by the authors.

Author biographies follow the reference list. Their grayscale portraits retain the original embedded pixel dimensions; all four exceed or meet 300 dpi at the displayed size.
