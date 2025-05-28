# !/bin/sh
ament_uncrustify --reformat include/
ament_uncrustify --reformat src/
echo "Code formatting completed using ament_uncrustify."