# required for docker build
if [ ! -d "${ROVERFLAKE_ROOT}/.git" ]; then
    echo "Not a git repository, skipping submodule update"
    exit 0
fi

git submodule update --init --recursive
