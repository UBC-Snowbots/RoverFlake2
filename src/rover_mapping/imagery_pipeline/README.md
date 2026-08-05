# imagery_pipeline — offline satellite tiles for mapviz

Turns imagery into an **offline** XYZ tile set that mapviz's `tile_map`
plugin uses as the base layer, plus a `mosaic.tif` the report exporter
draws on. No internet needed in the field. All scripts are ROS-free.

```
tile service ──► fetch_tiles.py ──┐
GeoTIFF ──► prepare_imagery.sh ───┴──► imagery/<site>/{tiles/, mosaic.tif}
                                              │
                 mapviz tile_map ◄── serve_tiles.sh (localhost:8000)
```

Normally you drive all of this through the package CLI:
`../rover fetch|sites|view`. The scripts here are what it calls.

## scripts/

- **fetch_tiles.py** — download a bbox from any XYZ tile service and build
  the mosaic. This is how the Drumheller set was made (Esri World Imagery;
  their terms require the attribution line, which `rover export` adds):

  ```bash
  ./scripts/fetch_tiles.py --name circ \
      --bbox 51.4422,-112.7293,51.4638,-112.6947 --zoom 13 19 \
      --url "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
  ```

- **prepare_imagery.sh** — your own GeoTIFFs (USGS Earth Explorer NAIP,
  UBC Abacus 10 cm orthos, ...) → the same layout. Higher quality than a
  tile service when you can get them; draw the download polygon generously
  since anything outside renders blank:

  ```bash
  ./scripts/prepare_imagery.sh ~/Downloads/ortho*.tif mysite 14 21
  ```

  Needs `gdal-bin` (>= 3.1 for `gdal2tiles --xyz`).

- **serve_tiles.sh** — tiny static HTTP server over `imagery/`, started
  automatically by the launch files. Base URL pattern:
  `http://localhost:8000/<site>/tiles/{level}/{x}/{y}.png`
  For viewing from another machine: `BIND=0.0.0.0 ./scripts/serve_tiles.sh`
  and use this host's IP in the base URL.

## imagery/ (gitignored — regenerate or copy from a teammate)

- `ubc_test/` — small Esri set around UBC; the bench-test default
  (`fake_gps` starts inside it).
- `drumheller/` — Esri set around the placeholder course center
  (51.4530, -112.7120), zoom 13–19. **Re-fetch around the real course
  center once known**: `../rover fetch circ <lat> <lon> 1.5`.
