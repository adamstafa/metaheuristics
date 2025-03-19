python3 generate_md.py
pandoc  -V geometry:"left=2cm, top=2cm, right=2cm, bottom=2cm" -V fontsize=9pt output/output.md operator_description.md -o output/report.pdf