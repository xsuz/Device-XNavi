#!/usr/bin/env bash

user=$(git config --get user.name)
email=$(git config --get user.email)

year=$(date +%Y)
repo_url=$(git remote get-url origin)
repo_name=$(basename -s .git "$repo_url")

device_name=${repo_name#Device-}


# modify kicad files

echo "Initializing KiCAD Library..."

cat circuit/Template/Template.kicad_pcb 	| sed 's/Template/'$device_name'/' > 'circuit/Template/'$device_name'.kicad_pcb'
cat circuit/Template/Template.kicad_prl 	| sed 's/Template/'$device_name'/' > 'circuit/Template/'$device_name'.kicad_prl'
cat circuit/Template/Template.kicad_pro 	| sed 's/Template/'$device_name'/' > 'circuit/Template/'$device_name'.kicad_pro'
cat circuit/Template/Template.kicad_sch 	| sed 's/Template/'$device_name'/' > 'circuit/Template/'$device_name'.kicad_sch'

rm circuit/Template/Template* -rf
mv 'circuit/Template' 'circuit/'$device_name

# initialize git submodule

echo "Updating submodules..."

git submodule update --init --remote

echo "Removing files..."

rm -r ./setup.sh ./README.md
