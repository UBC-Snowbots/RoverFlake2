#!/bin/bash
set -e

SRC_FILE="dds_profile_cyclone_template.xml"
DEST_FILE="dds_profile_cyclone.xml"

if [ -f "$DEST_FILE" ]; then
    echo "[⚠️  WARNING] $DEST_FILE already exists."
    read -p "Do you want to overwrite it? [y/N]: " confirm
    confirm=${confirm,,}  # lowercase
    if [[ "$confirm" != "y" && "$confirm" != "yes" ]]; then
        echo "[ABORTED] File was not overwritten."
        exit 0
    fi
fi

cat "$SRC_FILE" > "$DEST_FILE"
#$echo "[✅] $DEST_FILE written from $SRC_FILE."

#cat dds_profile_cyclone_zerotier.xml >> dds_profile_cyclone.xml
echo "dds_profile_cyclone.xml has been created just for you! Please kindly change the IP in the file to get it working. "

