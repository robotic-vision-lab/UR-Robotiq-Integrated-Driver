#!/usr/bin/env sh

# script folder; https://stackoverflow.com/a/4774063
SCRIPT_DIR="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

# build dir
SRC_HTML="$SCRIPT_DIR/build/html"
SRC_PDF="$SCRIPT_DIR/build/latex/rvl_driver_documentation.pdf"

# destinations
DRVER_DOC="$( cd -- "$LOCAL_DOC/../../UR-Robotiq-Integrated-Driver/documentation" >/dev/null 2>&1 ; pwd -P )"

# confirming paths
echo $SRC_HTML
while true; do
    read -p "Is generated HTML directory above correct [YyNn]? " yn
    case $yn in
        [Yy]* ) break;;
        [Nn]* ) echo "Aborted."; exit;;
        * ) echo "Please answer yes or no.";;
    esac
done

echo $SRC_PDF
while true; do
    read -p "Is generated PDF file above correct [YyNn]? " yn
    case $yn in
        [Yy]* ) break;;
        [Nn]* ) echo "Aborted."; exit;;
        * ) echo "Please answer yes or no.";;
    esac
done

echo "Destination is $DRVER_DOC"
while true; do
    read -p "Is the destination directory correct [YyNn]? " yn
    case $yn in
        [Yy]* ) break;;
        [Nn]* ) echo "Aborted."; exit;;
        * ) echo "Please answer yes or no.";;
    esac
done

exit

cp -r $SRC_HTML $DRVER_DOC
cp -r $SRC_PDF $DRVER_DOC

echo "Documentation updated."