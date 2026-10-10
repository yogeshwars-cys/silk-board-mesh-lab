# Silk Board Mesh Lab

A live simulation of evening-peak traffic on 1.2 km of Hosur Road into the Silk Board signal in Bengaluru, with a button that applies the RESILANE phone mesh. Applying it forks the road into two identical copies, so the comparison shows only what the mesh changed.

## Run it

Serve the folder and open the page:

```
python -m http.server 8000
```

Then open http://localhost:8000/index.html. Opening `index.html` straight from disk also works in most browsers.

## Files

- `index.html`: the page.
- `sim-core.js`: traffic model and mesh, with no DOM. The relay rules are ported from the RoadMesh app's `core/mesh/MeshEngine.kt`, road queries from `core/road/RoadQueries.kt` and speed advice from `core/road/RoadPicture.kt`. Keep them in step when those change.
- `check.mjs`: headless run with Node. `node check.mjs [seed] [minutes]` checks that two forks without the mesh stay identical, then prints the with/without comparison. `DEMAND`, `SHARE` and `HEED=slow,hazard` (`1`/`0`; `2` for slow means queue tails only) override the settings.

The model, its assumptions and measured results are on the page under "How this is modelled". It is not calibrated against field data; use it to compare designs under identical conditions.
