#!/usr/bin/env bash
#
# prepare_imagery.sh — Convert USGS Earth Explorer GeoTIFF(s) into an offline
# XYZ tile set that mapviz's tile_map plugin can display via a local server.
#
# Usage:
#   ./prepare_imagery.sh <input_dir_or_file> <output_name> [min_zoom] [max_zoom] [--srs EPSG:xxxx]
#
# Examples:
#   ./prepare_imagery.sh ~/Downloads/naip_download/ competition_site
#   ./prepare_imagery.sh site.tif competition_site 13 20
#   ./prepare_imagery.sh ~/Downloads/ubc_orthos/ ubc 14 21 --srs EPSG:3157
#
# Input:  a single GeoTIFF, or a directory containing .tif/.zip files
#         (Bulk Download zips from Earth Explorer are unzipped automatically).
# Output: imagery/<output_name>/tiles/{z}/{x}/{y}.png  + a mosaic.tif
#
# --srs   Assert a source projection on rasters that carry none. Needed for
#         imagery georeferenced only by a world file (.tfw/.sdw/.jgw), e.g.
#         the UBC Abacus orthophotos -> EPSG:3157 (NAD83(CSRS) / UTM 10N).
#         Only applied to rasters GDAL reports as having no CRS.
#
# Requires: gdal-bin, python3-gdal  (sudo apt install gdal-bin python3-gdal)

set -euo pipefail

# ---------------------------------------------------------------- arguments
if [[ $# -lt 2 ]]; then
    grep '^#' "$0" | head -24
    exit 1
fi

INPUT="$1"
NAME="$2"
shift 2

MIN_ZOOM=13
MAX_ZOOM=20
SRS=""
POSITIONAL=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --srs) SRS="${2:-}"; [[ -n "$SRS" ]] || { echo "ERROR: --srs needs a value (e.g. EPSG:3157)"; exit 1; }; shift 2 ;;
        --srs=*) SRS="${1#--srs=}"; shift ;;
        *) POSITIONAL+=("$1"); shift ;;
    esac
done
if [[ ${#POSITIONAL[@]} -ge 1 ]]; then MIN_ZOOM="${POSITIONAL[0]}"; fi
if [[ ${#POSITIONAL[@]} -ge 2 ]]; then MAX_ZOOM="${POSITIONAL[1]}"; fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_DIR="${REPO_ROOT}/imagery/${NAME}"
WORK_DIR="${OUT_DIR}/work"
TILE_DIR="${OUT_DIR}/tiles"
NPROC="$(nproc 2>/dev/null || echo 4)"

command -v gdalinfo >/dev/null || { echo "ERROR: gdal-bin not installed (sudo apt install gdal-bin python3-gdal)"; exit 1; }

mkdir -p "$WORK_DIR" "$TILE_DIR"
echo "==> Output: $OUT_DIR"

# ------------------------------------------------- 1. collect input rasters
echo "==> [1/5] Collecting input rasters"
SRC_DIR="$WORK_DIR/src"
mkdir -p "$SRC_DIR"

# Sidecars (world files, PAM .aux.xml) carry the georeferencing for imagery
# that has none embedded — they must travel with the raster or the mosaic
# lands at pixel coordinates instead of on the map.
SIDECARS=( -iname '*.tfw' -o -iname '*.tifw' -o -iname '*.wld' -o -iname '*.jgw'
           -o -iname '*.sdw' -o -iname '*.j2w' -o -iname '*.aux.xml' -o -iname '*.prj' )
RASTER_PATTERNS=( -iname '*.tif' -o -iname '*.tiff' -o -iname '*.jp2' )

if [[ -f "$INPUT" ]]; then
    case "$INPUT" in
        *.zip) unzip -o -q "$INPUT" -d "$SRC_DIR" ;;
        *)     cp "$INPUT" "$SRC_DIR/"
               # bring along any sidecar sitting next to it
               for ext in tfw tifw wld jgw sdw j2w prj aux.xml; do
                   if [[ -f "${INPUT%.*}.$ext" ]]; then cp "${INPUT%.*}.$ext" "$SRC_DIR/"; fi
                   if [[ -f "${INPUT}.$ext" ]];    then cp "${INPUT}.$ext"    "$SRC_DIR/"; fi
               done
               ;;
    esac
elif [[ -d "$INPUT" ]]; then
    find "$INPUT" -maxdepth 2 -iname '*.zip' -exec unzip -o -q {} -d "$SRC_DIR" \;
    find "$INPUT" -maxdepth 2 \( "${RASTER_PATTERNS[@]}" \) -exec cp {} "$SRC_DIR/" \;
    find "$INPUT" -maxdepth 2 \( "${SIDECARS[@]}" \) -exec cp {} "$SRC_DIR/" \;
else
    echo "ERROR: input '$INPUT' not found"; exit 1
fi

# MrSID is a proprietary format the stock Ubuntu GDAL cannot read. Catch it
# here with an actionable message instead of "no rasters found".
mapfile -t SIDS < <(find "$SRC_DIR" -iname '*.sid' | sort)
if [[ ${#SIDS[@]} -gt 0 ]] && ! gdalinfo --formats | grep -qi 'MrSID'; then
    cat >&2 <<'MRSID'
ERROR: input contains MrSID (.sid) files, which this GDAL build cannot read.
       (gdalinfo --formats shows no MrSID driver — the DSDK is proprietary and
       is not in the Ubuntu gdal-bin package.)

       Fix, cheapest first:
         1. Re-download the imagery as GeoTIFF/JPEG2000 if the source offers it.
         2. Decode with Extensis' free MrSID DSDK, then re-run this script:
              mrsidgeodecode -i FILE.sid -o FILE.tif -of tifg
            If the result has no CRS, pass it here, e.g.:
              --srs EPSG:3157      # NAD83(CSRS) / UTM 10N, UBC campus
MRSID
    printf '\n       Found:\n' >&2
    printf '         %s\n' "${SIDS[@]}" >&2
    exit 1
fi

mapfile -t RASTERS < <(find "$SRC_DIR" \( "${RASTER_PATTERNS[@]}" \) | sort)
[[ ${#RASTERS[@]} -gt 0 ]] || { echo "ERROR: no rasters found in input"; exit 1; }
echo "    Found ${#RASTERS[@]} raster(s):"
printf '      %s\n' "${RASTERS[@]}"

# ------------------------------------------- 1b. assert CRS where missing
# gdalwarp refuses to reproject a raster with no source projection. Imagery
# georeferenced by a world file alone (UBC Abacus orthos, ABMI sheets) hits
# this, so --srs stamps the projection on just those rasters.
NO_SRS=()
for r in "${RASTERS[@]}"; do
    if ! gdalsrsinfo -o proj4 "$r" 2>/dev/null | grep -q '+proj'; then
        NO_SRS+=("$r")
    fi
done
if [[ ${#NO_SRS[@]} -gt 0 ]]; then
    if [[ -z "$SRS" ]]; then
        echo "ERROR: ${#NO_SRS[@]} raster(s) have no projection and no --srs was given:" >&2
        printf '         %s\n' "${NO_SRS[@]}" >&2
        echo "       Re-run with e.g. --srs EPSG:3157 (check the .aux.xml/.prj for the right code)." >&2
        exit 1
    fi
    echo "    Stamping $SRS on ${#NO_SRS[@]} raster(s) with no CRS"
    for r in "${NO_SRS[@]}"; do
        gdal_edit.py -a_srs "$SRS" "$r"
    done
fi

# --------------------------------------------------------- 2. mosaic (VRT)
echo "==> [2/5] Building mosaic"
VRT="$WORK_DIR/mosaic.vrt"
gdalbuildvrt -overwrite "$VRT" "${RASTERS[@]}" >/dev/null

# ----------------------------------------- 3. force 3-band RGB byte output
# NAIP is often 4-band (R,G,B,NIR); gdal2tiles wants plain RGB(A).
echo "==> [3/5] Normalizing to RGB"
BANDS=$(gdalinfo "$VRT" | grep -c '^Band ')
RGB_VRT="$WORK_DIR/mosaic_rgb.vrt"
# A 4th band is NIR on NAIP (drop it) but alpha on some orthos (keep it —
# dropping it would turn already-transparent gaps into opaque black).
HAS_ALPHA=0
if [[ "$BANDS" -ge 4 ]] && gdalinfo "$VRT" | grep -q 'ColorInterp=Alpha'; then
    HAS_ALPHA=1
fi
if [[ "$BANDS" -ge 3 && "$HAS_ALPHA" -eq 1 ]]; then
    gdal_translate -q -of VRT -b 1 -b 2 -b 3 -b 4 -ot Byte "$VRT" "$RGB_VRT"
elif [[ "$BANDS" -ge 3 ]]; then
    gdal_translate -q -of VRT -b 1 -b 2 -b 3 -ot Byte "$VRT" "$RGB_VRT"
else
    # single-band (grayscale) input: expand so tiles are still valid PNGs
    gdal_translate -q -of VRT -ot Byte "$VRT" "$RGB_VRT"
fi
echo "    Input bands: $BANDS (alpha: $([[ $HAS_ALPHA -eq 1 ]] && echo yes || echo no))"

# ------------------------------------- 4. reproject to Web Mercator (3857)
# gdal2tiles can warp internally, but an explicit warp with bilinear
# resampling looks noticeably better for imagery and lets us keep the
# intermediate mosaic for the Phase 6 report exporter.
#
# -dstalpha (not -dstnodata) is what makes uncovered area transparent. With a
# nodata value instead, gaps between sheets and everything outside the imagery
# render as solid black tiles in mapviz. Alpha rules out JPEG/YCBCR
# compression, hence DEFLATE — lossless, and mosaic.tif is the Phase 6 report
# source, so it is worth the extra disk.
echo "==> [4/5] Reprojecting to EPSG:3857 (Web Mercator)"
MERC_TIF="$OUT_DIR/mosaic.tif"
# already has an alpha band? a second one would confuse gdal2tiles
WARP_ALPHA=(-dstalpha)
if [[ "$HAS_ALPHA" -eq 1 ]]; then WARP_ALPHA=(); fi
gdalwarp -q -overwrite \
    -t_srs EPSG:3857 \
    -r bilinear \
    -multi -wo NUM_THREADS="$NPROC" \
    -co COMPRESS=DEFLATE -co PREDICTOR=2 -co ZLEVEL=6 \
    -co TILED=YES -co BIGTIFF=IF_SAFER \
    "${WARP_ALPHA[@]}" \
    "$RGB_VRT" "$MERC_TIF"
gdalinfo "$MERC_TIF" | sed -n '/Size is/p;/Pixel Size/p' | sed 's/^/    /'

# ----------------------------------------------------- 5. cut XYZ tile set
# --xyz => OSM/WMTS "slippy" numbering, which is what mapviz tile_map's
# Custom WMTS Source expects ({level}/{x}/{y}.png).
echo "==> [5/5] Generating tiles (zoom ${MIN_ZOOM}-${MAX_ZOOM}, ${NPROC} processes)"
gdal2tiles.py --xyz \
    -z "${MIN_ZOOM}-${MAX_ZOOM}" \
    -r bilinear \
    -w none \
    --processes="$NPROC" \
    "$MERC_TIF" "$TILE_DIR"

TILE_COUNT=$(find "$TILE_DIR" -name '*.png' | wc -l)
SIZE=$(du -sh "$TILE_DIR" | cut -f1)

# Center of the imagery (handy for initialize_origin fallback)
CENTER=$(gdalinfo "$MERC_TIF" | grep '^Center' || true)

cat <<EOF

======================================================================
 DONE — $TILE_COUNT tiles ($SIZE) in:
   $TILE_DIR

 Next steps:
   1. Serve the tiles:   ./scripts/serve_tiles.sh $NAME
   2. Point mapviz at this tile set:
        ../rover view $NAME
      then:  ros2 launch launch/mapviz_offline.launch.xml site:=$NAME
   Base URL:  http://localhost:8000/${NAME}/tiles/{level}/{x}/{y}.png
   Max Zoom:  $MAX_ZOOM
   $CENTER
======================================================================
EOF
