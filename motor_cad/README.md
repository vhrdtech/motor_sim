# motor_cad

Motor geometry from CAD: STEP import into an assembly tree of parts, part classification by role (with a confidence
and the reasons), a scene description written as glTF binary (`.glb`), and parametric test motors written to STEP.
Pure Rust: [monstertruck](https://crates.io/crates/monstertruck) for the B-rep, STEP and meshing, no OpenCASCADE.
The Bevy view `motor_twin` loads either a STEP file or the `.glb` this crate writes.

```sh
motor_cad gen DIR                          # parametric test motors: outrunner_5065.step, stepper_nema17.step, ...
motor_cad import motor.step                # classify, print the report (--report FILE, --glb - writes motor.glb)
motor_cad init-overrides motor.step        # write motor.motor.toml with every part and its guessed role
motor_twin motor.step                      # look at it (just twin motor.step)
motor_twin motor.step --screenshot out.png --cutaway
```

## Roles

`shaft`, `rotor` (core, bell, hub), `magnet`, `stator` (core and teeth), `winding`, `bearing`, `housing` (case, end
caps, flanges, mounts, stator hub), `fastener`, `pcb`, `wiring` (cables, leads, connectors), `sensor` (encoder, hall,
sensor magnet), `other`. Shaft, rotor and magnets turn with the rotor unless told otherwise.

## The convention for our own models

Classification reads names first, colours second and geometry last. A model drawn to this convention classifies at
full confidence; geometry then only checks it (the report lists conflicts). Everything here is optional: an untagged
model still classifies from keywords, colours and geometry, with a lower confidence.

- **Tags in names.** Put the role in square brackets in the part or component name: `Bell [rotor]`,
  `Magnet [magnet N]`, `Stator core [stator]`, `Phase A coil [winding]`, `6802ZZ [bearing]`,
  `M3x6 [fastener rotating]`. A tag holds one role word and any of the modifiers `rotating`/`static` and `N`/`S`
  (magnet polarity facing the air gap). Other brackets (`[rev B]`) are skipped. Tags are English; untagged names are
  also searched for keywords in a few languages (English, German, French, Spanish, Russian, Chinese, ...).
- **Sub-assemblies.** A sub-assembly tagged `[rotor]` (or `[rotating]`) makes everything inside it turn, fasteners
  and balancing screws included, without making its children rotors: `Rotor [rotor]` holding `Bell`, `Magnet [magnet N]`
  and `Shaft` gives one rotor, magnets and a shaft, all rotating. A tag on any other sub-assembly is weak evidence
  for its children (`Bearings [bearing]`).
- **Instances for patterns.** Place repeated parts (magnets, teeth coils, screws) as instances of one component, not
  as copies: the file stays small, and the instance count is evidence (14 instances of one block on a ring are poles).
- **Colours.** Colour solids or faces by material; the importer reads STEP styled items on solids, else the majority
  colour of the faces. The colours `motor_cad gen` uses (`generate::colour`) are the reference: copper
  `#B87333`-ish for windings, red/blue for N/S magnets, mid grey for laminations, light grey for shafts and bearings,
  aluminium for housings, black for fasteners, green for PCBs. Any copper, magnet grey or steel tone counts as weak
  evidence.
- **Axis.** Draw the motor turning about Z, through the origin. Other axes work (the importer votes an axis from the
  round parts and the shaft), but Z is never wrong; when the vote is wrong, set it in the override file.
- **Units.** Any length unit the STEP file declares; the scene is in metres.

## Exporting STEP

- **SolidWorks:** File → Save As → STEP AP214 (or AP242). In Options: export the assembly structure, "Export face/edge
  properties" and colours on; one STEP file for the whole top assembly. A `.SLDASM` alone can't be read: it needs a
  STEP export first.
- **FreeCAD:** select the top `App::Part` or assembly → File → Export → STEP with colors (`*.step`). Keep the
  structure (Preferences → Import-Export → STEP: "Export legacy" off, scheme AP214 or AP242).
- Fused vendor models (one solid for the whole motor) import, but classify as one part; split them in CAD or draw the
  parts that matter.

## Overrides

Corrections live beside the model in `<model>.motor.toml` (`motor.step` → `motor.motor.toml`), written to start with
by `motor_cad init-overrides`. The first matching entry wins; `match` is a product name, an instance name or a path
`Assembly/Sub/Part`, with `*` for anything:

```toml
[axis]                     # optional, model units
origin = [0.0, 0.0, 0.0]
direction = [0.0, 0.0, 1.0]

[[part]]
match = "Body001"
role = "stator"

[[part]]
match = "*/Rotor/*screw*"
role = "fastener"
rotating = true
```

## The report

`motor_cad import` prints a Markdown report (`--report FILE` to save it): the axis and how it was found, slot and pole counts where geometry
finds them, and a table of every part with its role, confidence, rotating flag and the evidence (tag, keyword,
colour, geometry, override) with conflicts marked.
