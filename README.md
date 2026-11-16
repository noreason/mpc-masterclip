# MasterClip

A stereo sample-peak clipper for standalone Akai MPCs, by PluginBooth.

MasterClip provides Hard, Soft, Warm and Classic clipping curves; optional 2x/4x
oversampling; stereo linking; parallel mix; gain matching; factory presets; and
input, output and clipping meters. It controls sample peaks, not reconstructed
true peaks, and should not be presented as a true-peak limiter.

## Build with PluginBooth

Paste this repository's GitHub URL into **Creator Studio → Publish → Submit from
GitHub**. PluginBooth's supported SDK recipe builds ARM32 and ARM64 variants,
runs source analysis and the native host simulator, and validates the package.
Completed builds and reports appear in Creator Studio. A build with warnings may
show findings; these are visible evidence, not a safety certification. Hardware
benchmarks and dependency vulnerability scanning are separate checks and must
not be inferred from a successful build.

A build without failed checks can create a private listing draft. Confirm the
tested MPC models and review the description before submitting it for moderation.
Existing local device notes do not certify a newly generated binary. See the
current simulator finding below before expecting a draft from this version.

## Build locally

Use the inMusic MPC Plugin SDK pinned at
`febd2ca5bd9eb16c4a412837196e145566bc981b`:

```sh
git clone https://github.com/inmusic-software/mpc-plugin-builder-sdk.git ../mpc-plugin-builder-sdk
git -C ../mpc-plugin-builder-sdk checkout febd2ca5bd9eb16c4a412837196e145566bc981b
python3 ../mpc-plugin-builder-sdk/tools/build.py . --toolchain gcc
```

The GNU recipe requires a native C++17 compiler, `g++-arm-linux-gnueabihf` and
`g++-aarch64-linux-gnu`. PluginBooth provides these in its isolated builder.
The output folder is `build/PluginBooth - VST - MasterClip`.

The DSP and wrapper live in `plugins/masterclip/`; `src/MasterClip.cpp` is the
SDK entry translation unit. The checked-in skin needs no build-time graphics
libraries. Source tests are included under `plugins/masterclip/tests/`.

## MPC use

Requires MPC OS 3.10.1 or later with Enhancements and External Plugin Support
enabled. Copy the complete `PluginBooth - VST - MasterClip` folder into
`VST2 Plugins` at the top level of the internal drive. Keep the folder and UID
unchanged. Test a new build in a saved or disposable project before relying on it.

No samples, firmware, private device configuration or proprietary plugin code
are included.

## License

Original code and artwork: MIT, copyright 2026 PluginBooth. The build uses the
MIT-licensed inMusic MPC Plugin SDK; its notice is in `SDK-LICENSE.txt`. Both
notices are also included in `Content/Notices/` so SDK-built ZIPs carry them.

## Validation of this source publication

- Existing DSP suite: **42 passed, 0 failed**, under AddressSanitizer and
  UndefinedBehaviorSanitizer.
- Existing wrapper suite: **29 passed, 0 failed**, with the same sanitizers,
  including saved control state, malformed chunks, meters, presets and parameter
  append compatibility. Leak detection was disabled in the restricted test sandbox;
  these results do not constitute a leak scan.
- The production-style SDK recipe compiled ARM32 and ARM64, validated the manifest,
  UI assets and exported plugin entry point, and included both license notices.
- **Known generic simulator finding:** the pinned SDK simulator compares every
  parameter after rendering more audio following state restoration. MasterClip's
  live `In L` / `In R` meters change with the next block, so this check reports a
  state-restoration failure for them. The dedicated wrapper suite verifies that
  saved controls restore and live meters update as designed. This is an identified
  test limitation, not a suppressed pass: PluginBooth currently displays the runtime
  finding and holds automatic draft creation for this build.

These checks do not establish performance or reliability on MPC hardware. Keep
findings visible and review the generated report before distributing a build.
