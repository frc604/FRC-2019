#!/bin/sh

if [ $# -lt 1 ]; then
    echo "Usage: $0 <name of tag>"
    exit
fi

git add --all
git diff --staged --quiet
hasdiff=$?

if [ $hasdiff -eq 0 ]; then
  echo "No changes exist on the index"
else
  git commit -m "Tag WPILib deploying on $(date)"
fi

git tag $1

if hash ant; then
  ant
else
  echo "Code tagged and ready for deploy"
fi
