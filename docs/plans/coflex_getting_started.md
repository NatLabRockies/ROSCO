# COFLEX in ROSCO: a getting-started guide

This guide is a map of the C++ ROSCO controller (branch `c++`) for implementing the online half
of COFLEX (Lazzerini et al., WES 10, 1303, 2025). It also explains what makes a change easy to
merge later. The work is yours to take wherever you like. File:line references were checked
against commit `9441443f`, and every command was run on macOS arm64.

**What the controller has to do** (paper Sect. 5.2–5.4):

- Look up three schedules from the estimated wind speed `V̂`: feedforward torque `Q*_FF(V̂)`,
  feedforward pitch `β*_FF(V̂)`, and a speed set point `ω*(V̂)`.
- Add PI corrections to the feedforward torque and pitch. Both PIs act on the same speed error,
  `ω* − ω` (Eqs. 11–15).
- Low-pass filter the feedforward inputs.
- Hold pitch above a lower limit `j(V̂)`, a second offline schedule (Eq. 18).
- Blend the two loops with **set point smoothing** (Sect. 5.3). There is no region state
  machine, but there is a speed bias that saturates one PI while the other acts. The paper
  credits this to ROSCO's scheme.

The paper's wind speed estimator uses a `Cp(ω,V,β)` table, and ROSCO has nothing like it, so
start with ROSCO's EKF. The paper's results come from HAWC2 on the IEA 15 MW, not from ROSCO.
The authors' code is on Zenodo (10.5281/zenodo.11191546); I did not inspect it.

## 1. Build, run, test

You need `openfast_io` 5.x (`pyproject.toml:56`) and OpenFAST v5 on your `PATH`
(`environment.yml:27-28`). With `openfast_io` 4.x, the test-case `.fst` files fail to load
(`ValueError: ... '1E+06'`). CI builds its environment from `environment.yml`
(`.github/workflows/CI_rosco-compile.yml:126`), then runs
`pip install -e . --no-build-isolation --no-deps` (`:158`).

```bash
cmake -S rosco/controller -B rosco/controller/build
cmake --build rosco/controller/build
cmake --install rosco/controller/build                   # copies libdiscon.* into rosco/lib/
python Examples/Test_Cases/update_libdiscon_extension.py # point Test_Cases' ServoDyn at your lib
cd Examples && python 05_openfast_sim.py && cd ..        # 10 s IEA-15 UMaineSemi, ~11 s wall
pytest -v test/regression                                # CI's command (CI_rosco-compile.yml:188)
python test/regression/run_regression.py                 # "RESULT: ALL IDENTICAL — 7,260,000 ..."
```

Notes on these commands:

- **ServoDyn paths.** The committed ServoDyn files hard-code another machine's library path.
  CI runs the same fix-up script (`CI_rosco-compile.yml:166`). Don't commit the diffs it
  leaves behind.
- **Example 05.** It writes a tuned `Examples/DISCON.IN`, but OpenFAST reads the test case's
  own `IEA-15-240-RWT-UMaineSemi_DISCON.IN` (ServoDyn line 87). Outputs (`.outb`, `.RO.dbg`,
  `.RO.dbg2`) land in that test-case folder.
- **Regression suite.** It takes about 2 minutes: 66 passed, 1 skipped. The skipped test is
  the HDF5 check. Baselines exist only for `darwin-arm64` and `linux-x86_64`, and other
  platforms **skip rather than fail**, so read the `Platform:` line.
  `test/regression/README.md` explains how to read a failure.

## 2. Architecture tour

`DISCON()` (`rosco/controller/src/discon.cpp:82`) runs eight stages in order (`:112-131`).
Stages 3–7 run only when `iStatus >= 0` (`:120`). Paths below are relative to
`rosco/controller/src/`.

| Stage (`Stages/`) | Calls | Main outputs |
|---|---|---|
| 1 sensing | `ReadAvrSWAP` | measurements into `LocalVar` |
| 2 setup | config on first call; `SetParameters` → `CheckInputs` (`setparameters.cpp:55`) | `CntrPar`, initial `LocalVar` |
| 3 filtering | `PreFilterMeasuredSignals` | `GenSpeedF`, `BlPitchCMeasF`, `WE_Vw_F` |
| 4 estimation | `WindSpeedEstimator` | `WE_Vw` |
| 5 supervisory | `PowerControlSetpoints`, `Shutdown`, `Startup` | `PRC_R_*`, `PRC_Min_Pitch` |
| 6 setpoints | `SpeedSetpoints`, `TorqueStateMachine`, `SetpointSmoother` | `PC_SpdErr`, `VS_SpdErr`, `SS_DelOmegaF` |
| 7 actuators | `TorqueControl`, `PitchControl`, yaw/flap/cable/StC | `GenTq`, `PitCom` → `avrSWAP` |
| 8 output | `Debug`, `WriteRestartFile` | `.RO.dbg*`, checkpoint |

**Where state lives:**

- **Across timesteps.** Four file-scope statics (`discon.cpp:36-39`):
  - `CntrPar`: parameters, constant after load.
  - `LocalVar`: per-step signals.
  - `PerfData`: Cp tables.
  - `ExtDLL`: the external-controller buffer.
- **Filters, PIs and rate limiters.** These live in `ObjState`
  (`include/controller_objects.hpp:38`), never in function-local statics (`:15-20`).
- **Parameter definitions.** `ControlParameters` is generated from
  `rosco_registry/rosco_types.yaml`.
- **Per-step signals.** `LocalVariables` is hand-written in `include/vit_types.h:444-628`,
  mirrored in the YAML for logging.
- **Checkpoint.** The warm-restart field list is also hand-written (`include/restart_fields.h:71`).

## 3. The current set point path

These notes assume the base tuning: `VS_ControlMode=2`, `SS_Mode=1`, `PS_Mode=1`, `WE_Mode=2`
(`test/regression/fixtures/scenario_01.IN`).

- **Estimate** (`Estimators/windspeedestimator.cpp`).
  - I&I: `:87-95`. EKF: `:98-262`.
  - Otherwise, filtered hub-height wind: `:264-266`. The estimator also falls back to that when
    its inputs leave range (`:44-71`).
  - `WE_Vw_F` filters the *previous* step's `WE_Vw` (`Filters/prefiltermeasuredsignals.cpp:87-89`).
- **Pitch reference** (`Setpoints/speedsetpoints.cpp`):
  1. `PC_RefSpd·PRC_R_Speed` (`:9`).
  2. `PRC_Mode=1` table lookup (`:12-19`).
  3. The smoother raises it (`:22-26`).
  4. Error: `PC_SpdErr` (`:30`).
- **Torque reference** (same file):
  1. `TSR·V̂/R·N` (`:34-35`).
  2. `VS_FBP` tables (`:50-60`).
  3. Low-pass filter (`:66-68`).
  4. Tower-resonance exclusion (`:71-73`).
  5. Clamp to rated (`:76-78`).
  6. `PRC_Mode=1` table (`:81-85`).
  7. The smoother lowers it (`:88-90`).
  8. Minimum-speed floor (`:93`).
  9. Error: `VS_SpdErr` (`:96`).
- **Smoother** (`Setpoints/setpointsmoother.cpp:8-19`). It builds a bias from pitch above
  minimum and the power deficit, then filters it. It runs *after* `SpeedSetpoints`
  (`Stages/stage_6_setpoints.cpp:13-15`), so each step uses last step's bias.
- **Torque** (`Controllers/torquecontrol.cpp`).
  - Modes 2–4: one PI on `VS_SpdErr`, limited to `[VS_MinTq, VS_MaxTq]` (`:29-44`).
  - Then shutdown, saturation and the rate limit (`:89-107`), and the open-loop override
    (`:110-115`).
  - `VS_State` is read only by Kω² mode (`:46-82`).
- **Pitch** (`Controllers/pitchcontrol.cpp`).
  - Gains are scheduled on `BlPitchCMeasF` (`:30-33`).
  - The PI acts on `PC_SpdErr`, limited to `[PC_MinPit, PC_MaxPit]` (`:36-43`). It uses last
    step's `PC_MinPit`, because the min-pitch schedule is applied after it (`:64-69`).
  - Floating feedback is added after the PI (`:72-75`), then the command is saturated and rate
    limited (`:78-86`).
- **Min pitch.** `PS_BldPitchMin` is interpolated on `WE_Vw_F`, then the larger of that and
  `PRC_Min_Pitch` is used (`Setpoints/pitchsaturation.cpp:6-12`).
- **PI element.** It clamps both the integrator and the output to *absolute* limits
  (`ControlElements/picontroller.hpp:19-24`).

The regression baselines pin the one-step lags above. Don't "fix" them in passing.

## 4. Where COFLEX goes

| COFLEX | Closest existing piece | Where |
|---|---|---|
| `ω*(V̂)` | `PRC_Mode=1` speed table | `speedsetpoints.cpp:12-19, :81-85` |
| `β*_FF` added to the pitch PI | floating feedback added after the PI | `pitchcontrol.cpp:72-75` |
| `Q*_FF` added to the torque PI | none | `torquecontrol.cpp:38` |
| two PIs on `ω* − ω` | `genTqPI`, `pcPitComTPI` | `torquecontrol.cpp:38`, `pitchcontrol.cpp:41` |
| smoothing bias (Eqs. 16–19) | `SetpointSmoother` (power deficit, not torque deficit) | `setpointsmoother.cpp:10-17` |
| `j(V̂)` | `PS_Mode` min-pitch schedule | `pitchsaturation.cpp:6-8` |

A shape that stays small and reviewable:

1. **A new mode flag**, read in stage 6.
2. **A new function, in its own file under `Setpoints/`,** that looks up the schedules into new
   `LocalVar` fields. Call it from `stage_6_setpoints` behind `if (CntrPar.<Flag> > 0)`, before
   `SpeedSetpoints`.
3. **A guarded branch inside `SpeedSetpoints`** that sends `ω*` to both references, the way the
   `PRC_Mode == 1` branches do.
4. **Guarded feedforward blocks in stage 7**, inside `TorqueControl` and `PitchControl`. Pass
   each PI limits shifted by the feedforward term (`min − FF`, `max − FF`). Otherwise the
   integrator clamps against the wrong bound and winds up.
5. **Compatibility checks in `CheckInputs`**, following the precedent at
   `ReadSetParameters/checkinputs.cpp:88-90`.

**Features that could conflict:**

- `PRC_Mode` and `VS_FBP`: both have their own speed tables.
- `PS_Mode`: either a second min-pitch schedule or the natural home for `j(V̂)`.
- `SS_Mode`: reuse it or replace it.
- `TRA_Mode` and `F_VSRefSpdCornerFreq`: both move the torque reference.
- `VS_ControlMode=1`: Kω² has no PI to correct.
- `VS_ConstPower`: sets `VS_MaxTq`.
- `SU_Mode`, `SD_Mode`, `OL_Mode`: they override the commands.
- `IPC_SatMode`: saturates at `PC_MinPit`.
- `WE_Mode=0`: the "estimate" is just the anemometer.

## 5. Adding a parameter and a table, end to end

I ran this list in a throwaway worktree with a do-nothing flag. It touched 11 files and 30
regenerated fixtures, and the suite stayed ALL IDENTICAL.

1. **Registry entry.** Add the parameter under `ControlParameters` in `rosco_types.yaml`.
   - For a flag, copy `PS_Mode` (`:472`).
   - For a table, copy `PS_WindSpeeds`/`PS_BldPitchMin` (`:478-485`): `allocatable: True` plus
     an integer count like `PS_BldPitchMin_N`.
2. **Regenerate.** Run `python rosco/controller/rosco_registry/write_registry.py`.
   - It regenerates `rosco_types.hpp`, `rosco_types_io.cpp` (TOML loader), `IO/debug.cpp` and
     `Examples/DISCON_template.toml` (`write_registry.py:96-102`).
   - CI reruns it (`CI_rosco-compile.yml:93, :162`), so never hand-edit those files.
3. **Hand-edit `controlparameters_view_t`.** Add the same field in `vit_types.h:21-310`, or the
   build fails with `no member named '…' in 'controlparameters_view_t'`.
4. **DISCON.IN parser** (`ReadSetParameters/readcontrolparameterfilesub.cpp`).
   - Copy the PS lines (`:355-357`) or the PRC lines (`:318-323`).
   - A missing line reads as 0 (`:112-115`). That is why 0 must mean off: old input files keep
     working.
5. **Input checks.** Copy the PS block in `CheckInputs` (`checkinputs.cpp:463-474`). `interp1d`
   throws on x values that are not strictly increasing, and clamps outside the table
   (`Functions/interpolation.cpp:19-29`).
6. **New `LocalVariables` fields** go in three places:
   - the YAML `LocalVariables` section. Every scalar goes to `.RO.dbg2`; `dbg: true` also
     changes `.RO.dbg`'s columns.
   - the struct in `vit_types.h`, or `debug.cpp` won't compile.
   - `restart_fields.h`, if the field must survive a warm restart.
7. **Toolbox schema** (`rosco/toolbox/inputs/toolbox_schema.yaml`).
   - Add the flag with `default: 0`, like `PS_Mode` (`:170`).
   - Put the tables in the `DISCON` pass-through block (`:621`), like the PRC tables
     (`:916-935`).
8. **Toolbox controller.** Read the flag in `rosco/toolbox/controller.py` (next to `:63`).
9. **Toolbox writer** (`rosco/toolbox/utilities.py`).
   - Add the keys to `DISCON_dict` (flags from `:528`; PS tables at `:655-657`).
   - Add explicit lines to `write_DISCON` (flag near `:118`; table like `:250-252`).
   - The pass-through loop (`:707`) lets a tuning YAML's `controller_params: DISCON:` carry the
     offline COFLEXOpt tables, so no new tuning code is needed.
   - `VS_FBP_U/_Omega/_Tau` (`:197-199`) already write steady-state ω and τ against wind speed.
     They're a good model for the table layout.
10. **Mode coverage.** Add the flag to `MODES` in `test/regression/mode_coverage.py:47`. Every
    registry parameter ending in `Mode` must be listed.
11. **Fixtures.** Run `python test/regression/scenarios.py --write-fixtures`. Each fixture gains
    one line. `test_tuning.py` and `test_mode_coverage.py` fail until you do this.
12. **Full suite.** It must still print ALL IDENTICAL.

Keep tables inline in DISCON.IN. Both parsers already handle inline arrays. A separate file is
riskier: the TOML path never reads the open-loop file (`readcontrolparameterfilesub.cpp:530`
is DISCON.IN only).

## 6. A test case for the new mode

Follow "Adding a scenario" in `test/regression/README.md`:

1. **Scenario entry.** Add `Scenario(31, ...)` to `_SCENARIO_LIST` (`test/regression/scenarios.py:543`),
   with `patches` that set your flag and tables. Scenario 20 (`:755`) is a model.
   - `apply_patches` only rewrites lines the tuner already writes (`:202-215`), so step 9 above
     comes first.
2. **Register it.** Add 31 to `scenario_order` (`:929`) and `ALL_SCENARIOS`
   (`run_regression.py:40`).
3. **Fixtures and determinism.** Regenerate the fixtures, then run the scenario 5+ times and
   confirm the MD5s agree.
4. **Baselines.** Capture macOS with
   `run_regression.py --rebuild --scenario 31 --update-baseline`.
   - Linux comes from `fetch_linux_baselines.sh`, which dispatches a workflow on your push
     remote. So it works from a fork with Actions enabled and `gh auth login`.
5. **Check the diff.** `git status test/regression/baselines/` must list only scenario-31
   files. Also confirm the new baseline changes when you change the tables.

## 7. A suggested starting path

1. **Get green.** Build, run example 05, run the suite. *Check:* ALL IDENTICAL.
2. **A flag that does nothing** (section 5, flag only). *Check:* ALL IDENTICAL; fixtures gain
   one line each.
3. **Tables in, still unused.** Add `ω*`, `β*_FF`, `Q*_FF` against `V̂`, with input checks.
   *Check:* ALL IDENTICAL; a DISCON with the flag on loads.
4. **`ω*(V̂)` drives both references**, with no feedforward yet. *Check:* with the same table,
   it behaves like a `PRC_Mode=1` run.
5. **Feedforward torque and pitch**, with shifted PI limits, logged in `.RO.dbg2`. *Check:* in
   steady wind, with schedules from the plant's own Cp surface, the PI corrections settle near
   zero.
6. **Smoothing, `j(V̂)`, then scenario 31.** *Check:* the old scenarios stay identical and 31
   passes on both platforms.

## If you want this merged upstream

These points are advice, not rules. Changes that follow them are ones the maintainers can review
and merge. Sweeping changes likely aren't, even good ones.

- **Work inside the existing stages.** Add a function and guarded calls. Don't reorder stages,
  rename shared fields, or refactor neighbouring code.
  - A `CntrPar`/`LocalVar` name appears in the registry, both parsers, the checkpoint, the
    `.dbg` headers and the toolbox, so one rename ripples everywhere.
  - A refactor mixed into a feature can't be reviewed on its own.
- **Put new behaviour behind a flag that defaults to off.** With it off, the controller must do
  exactly what it does today. The DISCON.IN parser reads missing lines as 0, so a flag where 0
  means off costs existing users nothing.
- **Keep the baselines unchanged.** `pytest -v test/regression` compares 30 scenarios bit for
  bit.
  - A fixture gaining a line is expected. A moved baseline means your change leaked outside its
    mode.
  - Even reordering arithmetic in shared code moves bits (README, "Reading the evidence").
- **Ask before touching shared code.** Guarded blocks in `TorqueControl`/`PitchControl` are
  unavoidable. Open an issue first if you need to change any of these:
  - `PIController`
  - the `SetpointSmoother` formula
  - the wind speed estimator (the paper's estimator would be its own `WE_Mode` and its own PR)
  - filters shared with other modes
  - the stage order
- **Keep PRs small.** Steps 2–3 of section 7 make a good first PR on their own.

## Open questions for Dan

1. **Target repo.** Which repo and branch should the student fork and target? `origin` is
   `dzalkind/ROSCO` (`c++`). NREL/ROSCO `main` is still Fortran.
2. **Registry gap.** `REFACTOR_NOTES.md` says the registry updates everything, but every new
   parameter also needs a hand edit to `controlparameters_view_t`. That struct is used only by
   the generated `populate_view`/`sync_from_view`, which nothing calls. Hand-edit it, as with
   `OutputFormat`, or fix the generator first?
3. **Committed library paths.** All 8 committed Test_Cases ServoDyn files point `DLL_FileName`
   at `/Users/dzalkind/...`. Should they be committed with `update_libdiscon_extension.py --generic`?
4. **Environment.** I verified everything in your existing `rosco-c` env (`openfast_io` 5.0.0,
   OpenFAST v5.0.0), not a freshly created one. The base miniforge env (`openfast_io` 4.2)
   fails the suite.
5. **Smoother.** Eq. 16 uses a torque deficit; ROSCO's smoother uses a power deficit
   (`setpointsmoother.cpp:11-12`). Should the smoother gain a variant, or should COFLEX carry
   its own? Either way it's shared code.
6. **Min pitch.** Should `j(V̂)` reuse the `PS_Mode` tables through pass-through, or should
   COFLEX own that schedule?
7. **`PRC_R_Speed`.** `speedsetpoints.cpp:63` scales `VS_RefSpd` by `PRC_R_Speed`, and `:68`
   overwrites it, so `PRC_R_Speed` reaches the torque reference only through the clamp at `:77`.
   Is that intended? It matters if COFLEX copies the PRC pattern.
8. **Estimator scope.** Is the paper's `Cp(ω,V,β)` estimator in scope for this student?
9. **Old docs.** `docs/source/rosco.rst` and `how_to_contribute_code.rst` still describe the
   Fortran layout and will contradict this guide.
